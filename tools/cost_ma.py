#!/usr/bin/env python3

import json

def read():
  with open('stdout.log') as f:
    while True:
      line = f.readline().strip()
      if not line or line[-1] != '}':
        break

      sanitized = line.replace(',}','}').replace(',]',']')
      # import pdb; pdb.set_trace()
      record = json.loads(sanitized)
      yield record

read_gen = read()

print('{:>20} | {:>15} | {:>15}'.format('state', 'a_max', 'a_c_max'))
for record in read():
  costs = [[t['transition'], t['evaluation']] for t in record['transitions'] if t['transition'] == record['to']]
  for c in costs:
    row = '{:>20} | {:>15} | {:>15}'.format(c[0], c[1]['a_max'], c[1]['a_c_max'])
    print(row)

