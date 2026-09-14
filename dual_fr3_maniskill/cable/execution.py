"""Small device-side grid checks and bounded CUDA graph ownership for Warp 0.3."""
import ctypes
import warnings

import warp as wp


BOUNDS_CHUNK = wp.constant(128)


@wp.kernel
def bounds_chunks(q: wp.array(dtype=wp.vec3), partial: wp.array2d(dtype=float), count: int):
    chunk = wp.tid()
    for axis in range(3):
        lower = float(3.4028234663852886e38)
        upper = float(-3.4028234663852886e38)
        invalid = float(0.0)
        for offset in range(BOUNDS_CHUNK):
            index = chunk * BOUNDS_CHUNK + offset
            if index < count:
                value = q[index][axis]
                if not wp.abs(value) <= 3.4028234663852886e38:
                    invalid = 1.0
                lower = wp.min(lower, value)
                upper = wp.max(upper, value)
        partial[chunk, axis] = lower
        partial[chunk, axis + 3] = upper
        partial[chunk, axis + 6] = invalid


@wp.kernel
def bounds_summary(partial: wp.array2d(dtype=float), result: wp.array(dtype=float),
                   chunks: int, spacing: float, padding: int, nx: int, ny: int, nz: int):
    # Only this tiny summary crosses to the host. Match grid_layout's float32
    # span/division and 16-cell rounding, including exact allocation boundaries.
    invalid = float(0.0)
    grow = float(0.0)
    for axis in range(3):
        lower = float(3.4028234663852886e38)
        upper = float(-3.4028234663852886e38)
        for chunk in range(chunks):
            lower = wp.min(lower, partial[chunk, axis])
            upper = wp.max(upper, partial[chunk, axis + 3])
            invalid = wp.max(invalid, partial[chunk, axis + 6])
        result[axis] = lower
        result[axis + 3] = upper
        cells = (upper - lower) / spacing
        # Do not convert non-finite / unrepresentable coordinates to integers.
        if (not cells >= 0.0 or not cells < 1.e9
                or not wp.abs(lower) / spacing < 1.e9
                or not wp.abs(upper) / spacing < 1.e9):
            invalid = 1.0
        else:
            # At a float32 rounding boundary conservatively ask the host to
            # apply the original allocation rule to the six exact extrema.
            guarded = cells + (wp.abs(cells) + 1.0) * 1.e-6
            needed = ((int(wp.ceil(guarded)) + 2 * padding + 15) / 16) * 16
            available = nx
            if axis == 1:
                available = ny
            if axis == 2:
                available = nz
            if needed > available:
                grow = 1.0
    result[6] = grow
    result[7] = invalid


class DeviceBounds:
    def __init__(self, count, device):
        self.count, self.device = count, device
        self.chunks = (count + 127) // 128
        self.partial = wp.zeros((self.chunks, 9), dtype=float, device=device)
        self.summary = wp.zeros(8, dtype=float, device=device)

    def check(self, positions, spacing, padding, dims):
        wp.launch(bounds_chunks, dim=self.chunks,
                  inputs=[positions, self.partial, self.count], device=self.device)
        wp.launch(bounds_summary, dim=1,
                  inputs=[self.partial, self.summary, self.chunks, spacing, padding, *dims],
                  device=self.device)
        result = self.summary.numpy()
        if result[7] != 0:
            raise RuntimeError('MPM particle bounds became non-finite or unrepresentable')
        return result[:6].reshape(2, 3) if result[6] else None


class ConstraintGraphs:
    """Own at most one graph per ping-pong state for a single configuration.

    Captured kernels read changing poses/COM from persistent device arrays.
    A signature change retires both graphs before their buffers can be replaced.
    Capture records work without executing it; replay executes exactly once.
    """
    def __init__(self, enabled):
        self.enabled = enabled
        self.signature = None
        self.graphs = {}
        self.captures = 0
        self.replays = 0
        self.fallback_reason = None
        self.capturing = False

    def clear(self):
        if self.graphs:
            wp.synchronize()
            for graph in self.graphs.values():
                wp.context.runtime.core.cuda_graph_destroy(ctypes.c_void_p(graph))
            self.graphs.clear()
        self.signature = None

    def run(self, signature, state_key, record):
        if not self.enabled:
            record()
            return
        if signature != self.signature:
            self.clear()
            self.signature = signature
        if state_key not in self.graphs:
            if (wp.config.verify_cuda or not all(callable(getattr(wp, name, None))
                    for name in ('capture_begin', 'capture_end', 'capture_launch'))):
                self.enabled = False
                self.fallback_reason = 'CUDA graph API unavailable or verify_cuda enabled'
                warnings.warn(self.fallback_reason + '; using ordinary CUDA launches', RuntimeWarning)
                record()
                return
            # Compile/load before capture. Compilation or a failed recording is
            # a real error: do not conceal it by running possibly partial work.
            wp.capture_begin()
            self.capturing = True
            try:
                record()
            except BaseException:
                try:
                    graph = wp.capture_end()
                    wp.context.runtime.core.cuda_graph_destroy(ctypes.c_void_p(graph))
                except RuntimeError:
                    pass
                self.capturing = False
                raise
            try:
                self.graphs[state_key] = wp.capture_end()
            except RuntimeError as exc:
                # EndCapture closes an invalid/unsupported capture without
                # executing it. Ordinary launches remain the compatible path.
                self.enabled = False
                self.fallback_reason = str(exc)
            finally:
                self.capturing = False
            if not self.enabled:
                warnings.warn('CUDA graph capture unavailable: ' + self.fallback_reason
                              + '; using ordinary CUDA launches', RuntimeWarning)
                record()
                return
            self.captures += 1
        wp.capture_launch(self.graphs[state_key])
        self.replays += 1

    def __del__(self):
        # Normal scene shutdown calls clear explicitly; tolerate interpreter
        # teardown, when Warp's runtime may already have been unloaded.
        try:
            self.clear()
        except Exception:
            pass
