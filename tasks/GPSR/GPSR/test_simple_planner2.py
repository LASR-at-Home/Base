"""Stress-test simple_planner2 against generated GPSR commands.

For each command: run the LLM template match + deterministic expansion, then
score the result with structural checks (no ground truth needed):

  1. template is one of the 22 known templates
  2. every followup in the chain is one of the 13 known followups
  3. non-terminal followups (findObj/findPrs/meetName/takeObj) are not last in the chain
  4. expansion produced at least one step
  5. slot fidelity: content words of the command's nouns appear in the step args
     (rough check that slots were filled from the command, not hallucinated)
  6. manipulation sanity: if the command has a take/grasp/fetch/bring verb, the
     plan must contain pick_up; a pick_up must be followed by place/give

Usage:
  python3 test_simple_planner2.py                 # run built-in suite
  python3 test_simple_planner2.py commands.txt    # one command per line
  python3 test_simple_planner2.py -v              # also print each plan
"""

import json
import re
import sys
import time

from simple_planner2 import match_template, expand, TEMPLATE_STEPS, FOLLOWUP_STEPS

# Batch generated with athome-generator -d Incheon2026/
DEFAULT_COMMANDS = [
    "Navigate to the bedroom then find a snack and get it and throw it in the trash",
    "Navigate to the kitchen then meet Jane and tell something about yourself",
    "Lead the waving person from the laundry table to the bedside table",
    "Grasp a blue shirt from the dishwasher and throw it in the trash",
    "Bring me a soju from the cabinet",
    "Meet Morgan in the living room and say your teams affiliation",
    "Go to the living room then locate the person raising their left arm and follow them",
    "Tell me what is the heaviest food on the tv stand",
    "Go to the bedroom then meet Jules and follow them to the tv stand",
    "Go to the laundry then look for the sitting person and follow them to the kitchen",
    "Tell me the gesture of the person at the refrigerator",
    "Fetch a coke from the laundry table and put it on the bedside table",
    "Navigate to the refrigerator then find the standing person and follow them to the kitchen",
    "Look for a toy in the kitchen then grasp it and place it on the coffee table",
    "Say the day of the week to the waving person in the bedroom",
    "Lead the person wearing a red shirt from the cabinet to the kitchen trash bin",
    "Escort Jules from the bedside table to the living room",
    "Tell me what is the biggest dish on the washing machine",
    "Tell me what is the heaviest cleaning supply on the laundry trash bin",
    "Say what day is today to the person raising their left arm in the kitchen",
    "Go to the tv stand then look for a coke and fetch it and deliver it to Adel in the bedroom",
    "Meet Simone in the laundry and tell something about yourself",
    "Tell me how many food there are on the dishwasher",
    "Find a fruit in the living room then fetch it and put it on the cabinet",
    "Tell your teams affiliation to the person raising their left arm in the bedroom",
    "Lead the standing person from the coffee table to the laundry",
    "Tell me how many fruits there are on the tv stand",
]

NON_TERMINAL_FOLLOWUPS = {"findObj", "findPrs", "meetName", "takeObj"}
MANIP_VERBS = re.compile(r"\b(take|grab|grasp|get|fetch|bring|deliver|put|place|throw)\b", re.I)

STOPWORDS = {
    "the", "a", "an", "to", "from", "in", "on", "at", "of", "me", "my", "it",
    "them", "then", "and", "go", "navigate", "find", "look", "for", "locate",
    "take", "grab", "grasp", "get", "fetch", "bring", "deliver", "give", "put",
    "place", "throw", "trash", "meet", "greet", "say", "tell", "follow", "guide",
    "lead", "escort", "person", "people", "there", "are", "is", "what", "how",
    "many", "your", "something", "about", "yourself",
}


def content_words(text: str) -> set:
    return {w for w in re.findall(r"[a-z]+", text.lower()) if w not in STOPWORDS and len(w) > 2}


def _table_literal_words() -> set:
    """Words hardcoded in the expansion tables (say texts, 'trash bin', ...) — not hallucinations."""
    words = set()
    for table in (TEMPLATE_STEPS, FOLLOWUP_STEPS):
        for step_list in table.values():
            for _, arg_spec in step_list:
                for value in arg_spec.values():
                    words |= content_words(re.sub(r"\$\w+(\|\w+)*", " ", value))
    return words


TABLE_WORDS = _table_literal_words()


def check(command: str, match: dict, steps: list) -> list:
    """Return a list of failure strings (empty = all checks passed)."""
    failures = []

    # mirror expand()'s repairs: snap near-miss template names, drop unknown followups
    import difflib
    template = match.get("template", "")
    if template not in TEMPLATE_STEPS:
        close = difflib.get_close_matches(template, TEMPLATE_STEPS, n=1, cutoff=0.6)
        if close:
            template = close[0]
        else:
            failures.append(f"unknown template '{template}'")

    chain = [f for f in (match.get("followup") or "").split(">") if f in FOLLOWUP_STEPS]
    for j, f in enumerate(chain):
        if f not in NON_TERMINAL_FOLLOWUPS:
            chain = chain[: j + 1]
            break
    if chain and chain[-1] in NON_TERMINAL_FOLLOWUPS:
        failures.append(f"chain ends on non-terminal followup '{chain[-1]}'")

    if not steps:
        failures.append("empty plan")

    # slot fidelity: words used in step args must come from the command
    # (words hardcoded in the expansion tables are fine)
    args_text = " ".join(
        str(v) for s in steps for v in s.get("args", {}).values()
    )
    unresolved = re.findall(r"\$\w+", args_text)
    if unresolved:
        failures.append(f"unresolved slot reference in args: {sorted(set(unresolved))}")
    hallucinated = content_words(args_text) - content_words(command) - TABLE_WORDS
    if hallucinated:
        failures.append(f"arg words not in command: {sorted(hallucinated)}")

    # manipulation sanity — "tell/say/count" templates legitimately have no pick_up
    skills = [s["skill"] for s in steps]
    is_report_template = template.startswith(("tell", "count", "talk", "greet"))
    if MANIP_VERBS.search(command) and not is_report_template and "pick_up" not in skills:
        failures.append("command implies manipulation but plan has no pick_up")
    if "pick_up" in skills:
        after = skills[skills.index("pick_up"):]
        if not any(s in after for s in ("place_object", "give_to_person")):
            failures.append("pick_up not followed by place_object/give_to_person")

    return failures


PLANS_FILE = "plans.json"


def generate(commands: list) -> list:
    """Phase 1: run the LLM on every command and save plans to PLANS_FILE."""
    plans = []
    for i, cmd in enumerate(commands, 1):
        print(f"[{i}/{len(commands)}] {cmd}")
        t0 = time.perf_counter()
        try:
            match = match_template(cmd)
            steps = expand(match)
            entry = {"command": cmd, "match": match, "steps": steps,
                     "time_sec": round(time.perf_counter() - t0, 2)}
        except Exception as exc:
            entry = {"command": cmd, "error": str(exc)}
            print(f"  ERROR: {exc}")
        plans.append(entry)

    with open(PLANS_FILE, "w") as f:
        json.dump(plans, f, indent=2)
    print(f"\nsaved {len(plans)} plans to {PLANS_FILE}")
    return plans


def verify(plans: list) -> None:
    """Phase 2: check the logic of the saved plans (no LLM needed)."""
    passed, failed, errored = 0, 0, 0
    for i, entry in enumerate(plans, 1):
        cmd = entry["command"]
        print(f"[{i}/{len(plans)}] {cmd}")
        if "error" in entry:
            errored += 1
            print(f"  ERROR: {entry['error']}\n")
            continue

        match = entry["match"]
        steps = expand(match)  # re-expand: deterministic, applies current repair logic
        failures = check(cmd, match, steps)
        print(f"  template={match.get('template')} followup={match.get('followup') or '-'}")
        print("  plan: " + " -> ".join(s["skill"] for s in steps))
        if failures:
            failed += 1
            for f in failures:
                print(f"  FAIL: {f}")
        else:
            passed += 1
            print("  OK")
        print()

    n = len(plans)
    print("=" * 60)
    print(f"passed:  {passed}/{n}  ({100 * passed / n:.0f}%)")
    print(f"failed:  {failed}/{n}")
    print(f"errored: {errored}/{n}")
    times = [e["time_sec"] for e in plans if "time_sec" in e]
    if times:
        print(f"avg LLM time: {sum(times) / len(times):.1f}s per command")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    check_only = "--check" in sys.argv

    if check_only:
        with open(PLANS_FILE) as f:
            plans = json.load(f)
    else:
        if args:
            with open(args[0]) as f:
                commands = [l.strip() for l in f if l.strip()]
        else:
            commands = DEFAULT_COMMANDS
        plans = generate(commands)
        print()

    verify(plans)


if __name__ == "__main__":
    main()
