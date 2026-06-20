import sys


def read_input():
    lines = sys.stdin.read().splitlines()
    idx = 0

    def next_non_empty_line():
        nonlocal idx
        while idx < len(lines) and lines[idx] == "":
            idx += 1
        if idx >= len(lines):
            return ""
        line = lines[idx]
        idx += 1
        return line

    first = next_non_empty_line().split()
    N, D, H = map(int, first)

    users = []
    for _ in range(N):
        parts = next_non_empty_line().split()
        name = parts[0]
        budget = int(parts[1])
        energy = int(parts[2])
        k = int(parts[3])
        tags = set(parts[4:4 + k])
        users.append({
            "name": name,
            "budget": budget,
            "energy": energy,
            "tags": tags,
            "active": True,
        })

    A = int(next_non_empty_line())
    activities = {}
    for _ in range(A):
        parts = next_non_empty_line().split()
        aid = int(parts[0])
        activities[aid] = {
            "id": aid,
            "name": parts[1],
            "cost": int(parts[2]),
            "duration": int(parts[3]),
            "energy": int(parts[4]),
            "tag": parts[5],
        }

    E = int(next_non_empty_line())
    events = []
    for _ in range(E):
        # Keep the event text exactly as a single normalized line for the header.
        events.append(next_non_empty_line().strip())

    return N, D, H, users, activities, events


def clone_users(users):
    return [
        {
            "name": u["name"],
            "budget": u["budget"],
            "energy": u["energy"],
            "tags": set(u["tags"]),
            "active": u["active"],
        }
        for u in users
    ]


def fmt_day(day, ids, cost, sat):
    if not ids:
        return f"Day {day}: REST | cost=0 satisfaction=0"
    return f"Day {day}: {' '.join(map(str, sorted(ids)))} | cost={cost} satisfaction={sat}"


def better_choice(candidate, best):
    # candidate/best are tuples: (-satisfaction, cost, ids_tuple)
    return candidate < best


def choose_day(H, users, activities_by_id, chosen_before, blocked_tags):
    active_users = [u for u in users if u["active"]]
    if not active_users:
        return tuple(), 0, 0

    budget_limit = min(u["budget"] for u in active_users)
    energy_limit = min(u["energy"] for u in active_users)

    tag_points = {}
    for u in active_users:
        for tag in u["tags"]:
            tag_points[tag] = tag_points.get(tag, 0) + 1

    items = []
    for aid in sorted(activities_by_id):
        if aid in chosen_before:
            continue
        activity = activities_by_id[aid]
        if activity["tag"] in blocked_tags:
            continue
        # Any single activity that already exceeds a hard constraint can never appear.
        if (activity["cost"] > budget_limit or
                activity["energy"] > energy_limit or
                activity["duration"] > H):
            continue
        items.append((
            aid,
            activity["cost"],
            activity["energy"],
            activity["duration"],
            tag_points.get(activity["tag"], 0),
        ))

    n = len(items)
    suffix_points = [0] * (n + 1)
    for i in range(n - 1, -1, -1):
        suffix_points[i] = suffix_points[i + 1] + items[i][4]

    best_key = (0, 0, tuple())      # empty set: satisfaction 0, cost 0
    best_ids = tuple()
    best_cost = 0
    best_sat = 0

    sys.setrecursionlimit(1000000)

    def dfs(i, cost, energy, duration, sat, ids):
        nonlocal best_key, best_ids, best_cost, best_sat

        # If even taking every remaining positive-point activity cannot beat the
        # current satisfaction, only equal-score tie improvements are possible.
        # Equal-score tie improvements cannot beat the empty set unless sat > 0,
        # so keep this pruning conservative.
        if sat + suffix_points[i] < best_sat:
            return

        if i == n:
            candidate_key = (-sat, cost, ids)
            if better_choice(candidate_key, best_key):
                best_key = candidate_key
                best_ids = ids
                best_cost = cost
                best_sat = sat
            return

        aid, c, e, d, p = items[i]

        # Include first; this usually finds high-satisfaction candidates early.
        nc = cost + c
        ne = energy + e
        nd = duration + d
        if nc <= budget_limit and ne <= energy_limit and nd <= H:
            dfs(i + 1, nc, ne, nd, sat + p, ids + (aid,))

        # Exclude.
        dfs(i + 1, cost, energy, duration, sat, ids)

    dfs(0, 0, 0, 0, 0, tuple())
    return best_ids, best_cost, best_sat


def build_plan(start_day, D, H, users, activities, weather_blocks, plan):
    chosen_before = set()
    for day in range(1, start_day):
        if day in plan:
            chosen_before.update(plan[day][0])

    for day in range(start_day, D + 1):
        blocked = weather_blocks.get(day, set())
        ids, cost, sat = choose_day(H, users, activities, chosen_before, blocked)
        plan[day] = (ids, cost, sat)
        chosen_before.update(ids)


def apply_event(event_line, users_by_name, weather_blocks):
    parts = event_line.split()
    etype = parts[0]
    day = int(parts[1])

    if etype == "WEATHER":
        tag = parts[2]
        weather_blocks.setdefault(day, set()).add(tag)
    elif etype == "DROP":
        name = parts[2]
        if name in users_by_name:
            users_by_name[name]["active"] = False
    elif etype == "FATIGUE":
        name = parts[2]
        new_energy = int(parts[3])
        if name in users_by_name:
            users_by_name[name]["energy"] = new_energy
    elif etype == "BUDGET":
        name = parts[2]
        new_budget = int(parts[3])
        if name in users_by_name:
            users_by_name[name]["budget"] = new_budget

    return day


def plan_trip(N, D, H, users, activities, events):
    current_users = clone_users(users)
    users_by_name = {u["name"]: u for u in current_users}
    weather_blocks = {}
    plan = {}

    build_plan(1, D, H, current_users, activities, weather_blocks, plan)

    out = ["=== PLAN ==="]
    for day in range(1, D + 1):
        ids, cost, sat = plan[day]
        out.append(fmt_day(day, ids, cost, sat))

    for event_index, event_line in enumerate(events, 1):
        event_day = apply_event(event_line, users_by_name, weather_blocks)
        build_plan(event_day, D, H, current_users, activities, weather_blocks, plan)

        out.append(f"=== EVENT {event_index}: {event_line} ===")
        for day in range(event_day, D + 1):
            ids, cost, sat = plan[day]
            out.append(fmt_day(day, ids, cost, sat))

    return "\n".join(out) + "\n"


def main():
    N, D, H, users, activities, events = read_input()
    sys.stdout.write(plan_trip(N, D, H, users, activities, events))


if __name__ == "__main__":
    main()
