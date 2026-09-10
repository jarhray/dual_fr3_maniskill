"""Cover only allowed collision exclusions using PhysX's limited mask bits."""
from itertools import combinations


def collision_cliques(pairs, collidable_links, max_groups=31):
    nodes = set(collidable_links)
    edges = {frozenset(pair) for pair in pairs if len(pair) == 2 and set(pair) <= nodes}
    remaining = edges.copy()
    cliques = []
    while remaining:
        candidates = []
        for edge in sorted(remaining, key=lambda e: tuple(sorted(e))):
            clique = set(edge)
            for node in sorted(nodes - clique):
                if all(frozenset((node, other)) in edges for other in clique):
                    clique.add(node)
            covered = {frozenset(pair) for pair in combinations(clique, 2)} & remaining
            candidates.append((len(covered), tuple(sorted(clique)), covered))
        _, clique, covered = max(candidates, key=lambda item: (item[0], item[1]))
        cliques.append(clique)
        remaining -= covered
    if len(cliques) > max_groups:
        raise ValueError(f"SRDF requires {len(cliques)} collision mask groups (limit {max_groups})")
    return cliques
