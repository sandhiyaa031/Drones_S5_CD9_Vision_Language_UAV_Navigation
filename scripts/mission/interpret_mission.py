#!/usr/bin/env python3
"""Turn a natural-language instruction into a mission file (Phase 11).

Runs on the ground, before any flight. Prints the interpreted mission for
the operator to confirm; writes the mission file only when it is accepted
AND confirmed (interactive "yes", or --yes for scripted runs).

Usage:
  interpret_mission.py "Search the collapsed building for survivors and \
avoid falling debris" --out missions/from_text.json
Options:
  --interpreter llm|rules   language model (default) or keyword rules
  --endpoint URL --model M  OpenAI-compatible endpoint (default: the local
                            Ollama server and the CPU model)
  --record FILE             also write the full interpretation record
  --yes                     confirm without asking (recorded as such)
Exit status: 0 mission written; 2 refused; 3 interpreter failed; 4 not
confirmed.
"""

import argparse
import json
import os
import sys

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(WS, 'src', 'uav_autonomy'))

from uav_autonomy.mission_language import (  # noqa: E402
    STATUS_ACCEPTED, STATUS_REFUSED, interpret, load_catalogue, read_back)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('instruction')
    parser.add_argument('--out', required=True)
    parser.add_argument('--interpreter', choices=('llm', 'rules'),
                        default='llm')
    parser.add_argument('--endpoint', default=os.environ.get(
        'LLM_ENDPOINT', 'http://127.0.0.1:11434/v1'))
    parser.add_argument('--model', default=os.environ.get(
        'LLM_MODEL', 'gemma3-4b-cpu'))
    parser.add_argument('--places', default=os.path.join(
        WS, 'missions', 'places.json'))
    parser.add_argument('--record')
    parser.add_argument('--yes', action='store_true')
    args = parser.parse_args()

    catalogue = load_catalogue(args.places)
    result = interpret(args.instruction, catalogue, args.interpreter,
                       args.endpoint, args.model)
    print(read_back(result, catalogue))
    print(f'  [interpreter: {result.interpreter}, {result.latency_s:.1f} s]')
    confirmed = None
    status = 0
    if result.status == STATUS_ACCEPTED:
        if args.yes:
            confirmed = 'yes (--yes flag)'
        else:
            answer = input('Fly this mission? Type "yes" to accept: ')
            confirmed = 'yes (operator)' if answer.strip().lower() == 'yes' \
                else f'no ({answer.strip()!r})'
        if confirmed.startswith('yes'):
            with open(args.out, 'w') as handle:
                json.dump(result.mission_dict(), handle, indent=1)
                handle.write('\n')
            print(f'mission written to {args.out}')
        else:
            print('not confirmed; no mission written')
            status = 4
    else:
        status = 2 if result.status == STATUS_REFUSED else 3
    if args.record:
        with open(args.record, 'w') as handle:
            json.dump({
                'instruction': result.instruction,
                'interpreter': result.interpreter,
                'model': args.model if args.interpreter == 'llm' else None,
                'status': result.status, 'reasons': result.reasons,
                'fields': result.fields, 'raw_reply': result.raw,
                'latency_s': round(result.latency_s, 2),
                'confirmed': confirmed,
                'mission': result.mission_dict()
                if result.status == STATUS_ACCEPTED else None,
                'read_back': read_back(result, catalogue)}, handle, indent=1)
    sys.exit(status)


if __name__ == '__main__':
    main()
