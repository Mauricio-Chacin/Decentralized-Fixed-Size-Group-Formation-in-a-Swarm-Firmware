"""Decentralized fixed-size group formation simulation.

Implements the main mechanisms described in:
"Decentralized Force-Based Framework for Fixed-Size Group Formation
in Multi-Agent Systems" (Salik and Selmic).

Run: python K-formationTest_upgraded.py
Dependencies: numpy

This is a 2-D, discrete-time feasibility demonstrator. It uses a shared
simulation loop to emulate local sensing; it does not implement real radio
communication or physically distributed execution.
"""

from __future__ import annotations

import hashlib
import math
import os
import random
import sys
import time
import uuid
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Simulation parameters (adjust these for experiments)
# ---------------------------------------------------------------------------
SHAPE = 20                 # Width and height of the square world/grid
NUM_AGENTS = 14             # Total number of agents
K = 3                       # Required number of agents in each group
R_SENSE = 4.0               # Local sensing radius
DT = 0.12                   # Simulation timestep
FRAME_DELAY = 0.08          # Delay between terminal refreshes (seconds)
MAX_STEPS = 2500            # Maximum simulation duration
RANDOM_SEED = 42            # Reproducible initial conditions and noise

# Free-agent attraction / repulsion parameters from the paper's force model.
ALPHA = 0.9                 # Attraction gain
BETA = 1.8                  # Repulsion gain; should be greater than ALPHA
R_EQ = 1.5                  # Desired free-agent equilibrium distance
R_REP = R_EQ / (1.0 - ALPHA / BETA)  # Derived repulsion cutoff (paper)

# Other force-model parameters.
BETA_GROUP = 3.0             # Repulsion between different locked groups
K_COH = 1.2                  # Same-group spring stiffness
C_DAMP = 0.8                 # Same-group damping coefficient
GAMMA = 0.18                 # Velocity-alignment gain
NOISE_SIGMA = 0.10           # Random exploration/noise strength
TARGET_GAIN = 0.0            # Set > 0 to enable attraction to TARGET
TARGET: Optional[np.ndarray] = None  # Example: np.array([10.0, 10.0])
V_MAX = 2.0                  # Numerical speed limit
WORLD_MARGIN = 0.25          # Keep agents inside the displayed world

# Logistic penalty steepness. The paper calls this parameter r.
LOGISTIC_RATE = 4.0

# ASCII symbols used to render groups. Each group receives a different symbol.
# Avoid whitespace and the free-agent marker '1'.
GROUP_SYMBOLS = list("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz") + list("234567890@#$%&*+=?")

random.seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)

# ---------------------------------------------------------------------------
# Agent state
# ---------------------------------------------------------------------------
Agents: List[Dict] = []
NextGroupSymbol = 0


def unit_vector(vector: np.ndarray) -> np.ndarray:
    """Return a unit vector, or a zero vector if its length is nearly zero."""
    magnitude = float(np.linalg.norm(vector))
    if magnitude < 1e-9:
        return np.zeros(2, dtype=float)
    return vector / magnitude


def distance(a: Dict, b: Dict) -> float:
    """Euclidean distance between two agents."""
    return float(np.linalg.norm(b["pos"] - a["pos"]))


def logistic_penalty(free_neighbor_count: int, agent: Dict) -> float:
    """Density-dependent penalty D(rho), as defined in the paper.

    rho is the number of *unsaturated neighbors*, not including the agent.
    D approaches zero in sparse neighborhoods and one in dense neighborhoods.
    """
    exponent = -agent["rate"] * (((free_neighbor_count + 1) / agent["K"]) - 1)
    # Clipping prevents overflow in exp for unusually large parameter values.
    exponent = float(np.clip(exponent, -60.0, 60.0))
    return 1.0 / (1.0 + math.exp(exponent))


def neighborhood(agent: Dict, agents: Sequence[Dict], free_only: bool = False) -> List[Dict]:
    """Return agents visible within R_SENSE, using local metric sensing."""
    result = []
    for other in agents:
        if other["id"] == agent["id"]:
            continue
        if free_only and other["group_id"] is not None:
            continue
        if distance(agent, other) < agent["r_sense"]:
            result.append(other)
    return result


def initialize_agents(number: int) -> None:
    """Create agents with persistent UUIDs and proposed group UUIDs."""
    for agent_id in range(number):
        Agents.append({
            "id": agent_id,
            "uuid": str(uuid.uuid4()),
            # Each agent proposes a group UUID. Hash selection chooses one
            # proposal deterministically for a qualifying K-agent clique.
            "proposed_group_uuid": str(uuid.uuid4()),
            "pos": np.array([
                random.uniform(1.0, SHAPE - 2.0),
                random.uniform(1.0, SHAPE - 2.0),
            ], dtype=float),
            "vel": np.random.uniform(-0.15, 0.15, size=2),
            "group_id": None,
            "group_symbol": None,
            "K": K,
            "r_sense": R_SENSE,
            "rate": LOGISTIC_RATE,
            "D": 0.0,
            "density": 0,
        })


def hash_select_group_id(members: Sequence[Dict]) -> str:
    """Select a shared group UUID using the paper's hash-based rule.

    For each member i, compute SHA-256(encode(agent_UUID) ||
    encode(proposed_group_UUID)). The member with the lexicographically
    smallest digest wins; agent UUID breaks an exact digest tie. Its proposed
    group UUID becomes the common group identifier.

    Every member of a qualifying clique has the same member set, so this
    calculation produces the same answer locally for all members.
    """
    ranked = []
    for member in members:
        payload = (member["uuid"] + member["proposed_group_uuid"]).encode("utf-8")
        digest = hashlib.sha256(payload).hexdigest()
        ranked.append((digest, member["uuid"], member["proposed_group_uuid"]))
    ranked.sort(key=lambda item: (item[0], item[1]))
    return ranked[0][2]


def allocate_group_symbol() -> str:
    """Return a unique printable ASCII symbol for the next formed group."""
    global NextGroupSymbol
    if NextGroupSymbol >= len(GROUP_SYMBOLS):
        # A visible fallback; the simulation should rarely create this many.
        raise RuntimeError("Not enough unique ASCII group symbols configured.")
    symbol = GROUP_SYMBOLS[NextGroupSymbol]
    NextGroupSymbol += 1
    return symbol


def find_qualifying_cliques(agents: Sequence[Dict]) -> List[List[Dict]]:
    """Find disjoint K-agent cliques satisfying the paper's local trigger.

    A candidate is valid only if every member is free, each member sees
    exactly K-1 free neighbors, and every pair in the candidate is within
    R_SENSE. This preserves the paper's complete-clique assumption.
    """
    free_agents = [a for a in agents if a["group_id"] is None]
    candidates = {}

    for agent in free_agents:
        free_neighbors = neighborhood(agent, agents, free_only=True)
        if len(free_neighbors) != K - 1:
            continue
        members = [agent] + free_neighbors
        member_ids = frozenset(member["id"] for member in members)
        # Require a complete graph: every pair must be mutually in range.
        is_clique = all(
            distance(members[i], members[j]) < R_SENSE
            for i in range(K) for j in range(i + 1, K)
        )
        if is_clique:
            candidates[member_ids] = members

    # Resolve any overlapping candidates deterministically. Under the exact
    # neighbor-count/clique condition, overlaps are uncommon; this guard keeps
    # a single agent from being assigned to two groups in one timestep.
    accepted = []
    used_ids = set()
    for member_ids in sorted(candidates, key=lambda ids: tuple(sorted(ids))):
        if not (member_ids & used_ids):
            accepted.append(candidates[member_ids])
            used_ids.update(member_ids)
    return accepted


def form_groups(agents: Sequence[Dict]) -> int:
    """Lock qualifying cliques and assign a shared hash-selected group ID."""
    formed = 0
    for members in find_qualifying_cliques(agents):
        group_id = hash_select_group_id(members)
        symbol = allocate_group_symbol()
        for member in members:
            member["group_id"] = group_id
            member["group_symbol"] = symbol
            member["vel"][:] = 0.0
        formed += 1
    return formed


def pairwise_force(agent: Dict, other: Dict) -> np.ndarray:
    """Compute one of the five pairwise interaction cases from the paper.

    The direction vector points from this agent toward the other agent.
    A negative coefficient therefore pushes this agent away from the other.
    """
    delta = other["pos"] - agent["pos"]
    d = float(np.linalg.norm(delta))
    if d < 1e-9:
        # Deterministic tiny separation direction for coincident positions.
        angle = (agent["id"] * 2.399963 + other["id"]) % (2.0 * math.pi)
        direction = np.array([math.cos(angle), math.sin(angle)])
        d = 1e-6
    else:
        direction = delta / d

    same_is_free = agent["group_id"] is None
    other_is_free = other["group_id"] is None

    # Case 1: free-free; attraction and repulsion are density-modulated.
    if same_is_free and other_is_free:
        attraction = ALPHA * (1.0 - agent["D"]) * direction
        repulsion_strength = max(0.0, (R_REP - d) / R_REP)
        repulsion = -BETA * agent["D"] * repulsion_strength * direction
        return attraction + repulsion

    # Cases 2 and 3: free-locked or locked-free; pure exclusion.
    if same_is_free != other_is_free:
        repulsion_strength = max(0.0, (R_REP - d) / R_REP)
        return -BETA * repulsion_strength * direction

    # Case 4: locked agents belonging to the same group; spring-damper.
    if agent["group_id"] == other["group_id"]:
        relative_velocity = agent["vel"] - other["vel"]
        spring = K_COH * (d - R_EQ) * direction
        damping = -C_DAMP * float(np.dot(relative_velocity, direction)) * direction
        return spring + damping

    # Case 5: locked agents from different groups; strong repulsion.
    repulsion_strength = max(0.0, (R_REP - d) / R_REP)
    return -BETA_GROUP * repulsion_strength * direction


def calculate_forces(agents: Sequence[Dict]) -> List[np.ndarray]:
    """Calculate all forces from a shared snapshot before moving any agent."""
    forces = []
    for agent in agents:
        # Locked agents still receive pairwise forces: same-group cohesion and
        # separation from free agents / other groups.
        visible = neighborhood(agent, agents, free_only=False)
        interaction_force = np.zeros(2, dtype=float)
        for other in visible:
            interaction_force += pairwise_force(agent, other)

        motive_force = np.zeros(2, dtype=float)
        if agent["group_id"] is None:
            # Alignment: match the mean velocity of locally visible agents.
            if visible:
                mean_velocity = np.mean([other["vel"] for other in visible], axis=0)
                motive_force += GAMMA * (mean_velocity - agent["vel"])

            # Density-centroid attraction is the paper's local target when
            # there are unsaturated neighbors. Isolated agents instead explore.
            free_neighbors = neighborhood(agent, agents, free_only=True)
            if free_neighbors:
                centroid = np.mean([other["pos"] for other in free_neighbors], axis=0)
                if TARGET is None:
                    local_target = centroid
                else:
                    local_target = 0.5 * TARGET + 0.5 * centroid
                motive_force += TARGET_GAIN * (local_target - agent["pos"])
            elif TARGET is not None:
                motive_force += TARGET_GAIN * (TARGET - agent["pos"])

            # Small Gaussian noise provides exploration for free agents.
            motive_force += np.random.normal(0.0, NOISE_SIGMA, size=2)

        forces.append(interaction_force + motive_force)
    return forces


def update_density_and_penalty(agents: Sequence[Dict]) -> None:
    """Update free-neighbor density and logistic penalty for every free agent."""
    for agent in agents:
        if agent["group_id"] is not None:
            agent["density"] = 0
            agent["D"] = 1.0
            continue
        free_neighbors = neighborhood(agent, agents, free_only=True)
        agent["density"] = len(free_neighbors)
        agent["D"] = logistic_penalty(agent["density"], agent)


def advance_agents(agents: Sequence[Dict], forces: Sequence[np.ndarray]) -> None:
    """Apply discrete Euler updates using the same force snapshot."""
    for agent, force in zip(agents, forces):
        acceleration = force
        agent["vel"] = agent["vel"] + DT * acceleration
        speed = float(np.linalg.norm(agent["vel"]))
        if speed > V_MAX:
            agent["vel"] = agent["vel"] * (V_MAX / speed)
        agent["pos"] = agent["pos"] + DT * agent["vel"]

        # Reflect at world boundaries so agents remain visible on the grid.
        for axis in range(2):
            low = WORLD_MARGIN
            high = SHAPE - 1 - WORLD_MARGIN
            if agent["pos"][axis] < low:
                agent["pos"][axis] = low
                agent["vel"][axis] = abs(agent["vel"][axis])
            elif agent["pos"][axis] > high:
                agent["pos"][axis] = high
                agent["vel"][axis] = -abs(agent["vel"][axis])


def render_grid(agents: Sequence[Dict]) -> List[str]:
    """Create a character grid; free agents are '1', groups use unique glyphs."""
    grid = [["." for _ in range(SHAPE)] for _ in range(SHAPE)]
    occupants: Dict[Tuple[int, int], List[Dict]] = {}

    for agent in agents:
        # Map continuous coordinates to the nearest terminal cell.
        x = int(round(float(agent["pos"][0])))
        y = int(round(float(agent["pos"][1])))
        x = max(0, min(SHAPE - 1, x))
        y = max(0, min(SHAPE - 1, y))
        occupants.setdefault((x, y), []).append(agent)

    for (x, y), cell_agents in occupants.items():
        # If agents share a cell, show '+' so an overlap is not hidden.
        if len(cell_agents) > 1:
            symbols = {a["group_symbol"] if a["group_symbol"] else "1" for a in cell_agents}
            grid[y][x] = next(iter(symbols)) if len(symbols) == 1 else "+"
        else:
            agent = cell_agents[0]
            grid[y][x] = agent["group_symbol"] if agent["group_symbol"] else "1"

    border = "+" + "-" * SHAPE + "+"
    lines = [border]
    lines.extend("|" + "".join(row) + "|" for row in grid)
    lines.append(border)
    return lines


def terminal_supports_ansi() -> bool:
    """Best-effort check for ANSI cursor control support."""
    if os.name != "nt":
        return sys.stdout.isatty()
    # Windows Terminal and modern PowerShell support ANSI escape sequences.
    return sys.stdout.isatty() and ("WT_SESSION" in os.environ or "ANSICON" in os.environ
                                    or "TERM" in os.environ)


def draw_dashboard(step: int, formed_this_step: int, started: float,
                   use_ansi: bool, first_draw: bool) -> None:
    """Refresh a fixed dashboard by moving the cursor home, not clearing it."""
    grid_lines = render_grid(Agents)
    groups: Dict[str, List[int]] = {}
    for agent in Agents:
        if agent["group_symbol"] is not None:
            groups.setdefault(agent["group_symbol"], []).append(agent["id"])

    grouped_count = sum(len(ids) for ids in groups.values())
    free_count = NUM_AGENTS - grouped_count
    elapsed = time.time() - started

    output = []
    output.append("DECENTRALIZED K-FORMATION | live simulation")
    output.append(f"Step: {step:5d}   Time: {step * DT:7.2f}s   Groups: {len(groups):2d}   "
                  f"Grouped agents: {grouped_count:2d}   Free agents: {free_count:2d}")
    output.append(f"K={K}  Rsense={R_SENSE:.1f}  Rrep={R_REP:.2f}  "
                  f"D range: {min(a['D'] for a in Agents):.2f}..{max(a['D'] for a in Agents):.2f}")
    output.append("")
    output.extend(grid_lines)
    output.append("")
    output.append("Legend: 1 = free agent | A, B, C... = members of the same locked group | . = empty")
    output.append("         + = multiple different occupants rounded into the same grid cell")
    if groups:
        group_text = "  ".join(f"{symbol}: agents {','.join(map(str, ids))}"
                               for symbol, ids in sorted(groups.items()))
        output.append("Groups: " + group_text)
    else:
        output.append("Groups: none formed yet")
    output.append(f"New groups this step: {formed_this_step}   Wall time: {elapsed:.1f}s")
    output.append("Press Ctrl+C to stop.")

    if use_ansi:
        # Cursor-home overwrites the same dashboard area; no scrolling grid.
        sys.stdout.write("\033[H" + "\n".join(output) + "\n")
    else:
        # Non-interactive terminals cannot reliably reposition the cursor.
        # Print snapshots only in that case rather than emitting broken escapes.
        sys.stdout.write("\n".join(output) + "\n" + "=" * 60 + "\n")
    sys.stdout.flush()


def main() -> None:
    """Run the simulation until MAX_STEPS or until interrupted by the user."""
    initialize_agents(NUM_AGENTS)
    update_density_and_penalty(Agents)

    use_ansi = terminal_supports_ansi()
    if use_ansi:
        # Reserve the dashboard area once. Later frames return to its first line.
        print("\n" * (SHAPE + 20), end="")
        sys.stdout.write("\033[H")

    start_time = time.time()
    try:
        for step in range(MAX_STEPS + 1):
            # Group formation is evaluated from the current, unchanged snapshot.
            formed = form_groups(Agents)
            update_density_and_penalty(Agents)
            forces = calculate_forces(Agents)
            advance_agents(Agents, forces)
            update_density_and_penalty(Agents)

            draw_dashboard(step, formed, start_time, use_ansi, step == 0)
            time.sleep(FRAME_DELAY)

            # Stop naturally only when every agent is locked. If fewer than K
            # agents remain free, they stay unsaturated and continue moving,
            # matching the paper's stated remainder-agent behavior.
            free_count = sum(a["group_id"] is None for a in Agents)
            if free_count == 0:
                break
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")

    total_groups = len({a["group_id"] for a in Agents if a["group_id"] is not None})
    total_grouped = sum(a["group_id"] is not None for a in Agents)
    print(f"\nFinal result: {total_groups} groups, {total_grouped}/{NUM_AGENTS} agents grouped, "
          f"{NUM_AGENTS - total_grouped} agents remaining free.")


if __name__ == "__main__":
    main()
