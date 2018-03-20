from yaml import load, dump
from yaml import CLoader as Loader, CDumper as Dumper

f = open('lights-data/train.yaml', 'r')
data = load(f, Loader=Loader)

labeled = {}

colors = ['Red', 'Yellow', 'Green']

def box_color(boxes):
  for color in colors:
    if all(box['label'].find(color) >= 0 for box in boxes):
      return color

  return None

def store_label(item):
  if len(item['boxes']) == 0:
    labeled[item['path']] = 'None'
    return

  color = box_color(item['boxes'])

  if color != None:
    labeled[item['path']] = color

for item in data:
  store_label(item)

with open('file_labels.csv', 'w') as f:
  for path, label in labeled.iteritems():
    f.write(path)
    f.write(',')
    f.write(label)
    f.write("\n")
