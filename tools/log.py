import json

def get_color(color_idx = -1):
  colors = ['black','gray','navy','r','g','b','y','m','c'];
  while True:
    color_idx = (color_idx + 1) % (len(colors) - 1)
    yield colors[color_idx]

def read(path):
  with open(path) as f:
    while True:
      line = f.readline().strip()
      if not line or line[-1] != '}':
        break

      sanitized = line.replace(',}','}').replace(',]',']')
      # import pdb; pdb.set_trace()
      record = json.loads(sanitized)
      yield record
