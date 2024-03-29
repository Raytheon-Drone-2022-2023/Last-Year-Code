import cv2
import json
from sys import stdout

ANNOTATIONS_FILE = 'annotations.json' # File generated by SuperAnnotate
IMAGES_DIRECTORY = 'data/High_Shade/' # Slash at end
LOGO_BBOXES_FILE = 'data/positive.dat'
LOGO_OUTPUT_PATH = 'data/logos/output' # Slash at end
BACKGROUNDS_PATH = 'data/backgrounds/output/' # Slash at end

with open(ANNOTATIONS_FILE) as f:
  data = json.load(f)

logo_locations = {}

print(f'Reading annotations')
for obj in data:
  if '.jpg' not in obj: continue
  marked_points = data[obj]['instances'][0]['points'][:8]
  marked_points = [(round(marked_points[i]), round(marked_points[i+1])) for i in range(0, len(marked_points), 2)]

  min_x = min([x[0] for x in marked_points])
  max_x = max([x[0] for x in marked_points])
  min_y = min([x[1] for x in marked_points])
  max_y = max([x[1] for x in marked_points])

  logo_locations[obj] = (min_x, min_y, max_x, max_y)

output_file = open(f'{LOGO_BBOXES_FILE}', 'w+')

print(f'\nGetting logos')
logo_count = 0
for pic in logo_locations:
  logo_count += 1
  stdout.write(f'\u001b[20D{logo_count}/{len(logo_locations)}')
  stdout.flush()
  img = cv2.imread(f'{IMAGES_DIRECTORY}/{pic}', cv2.IMREAD_COLOR)
  min_x, min_y, max_x, max_y = logo_locations[pic]
  logo = img[min_y:max_y,min_x:max_x]
  x, y, w, h = int((min_x + max_x)/2), int((min_y + max_y)/2), max_x - min_x, max_y - min_y
  output_file.write(f'logos/{pic} 1 {x} {y} {w} {h}\n')
  cv2.imwrite(f'{LOGO_OUTPUT_PATH}/{pic}', img)
  cv2.imwrite(f'{BACKGROUNDS_PATH}/{"_0.".join(pic.split("."))}', img[:min_y,:])
  cv2.imwrite(f'{BACKGROUNDS_PATH}/{"_1.".join(pic.split("."))}', img[max_y:,:])
  cv2.imwrite(f'{BACKGROUNDS_PATH}/{"_2.".join(pic.split("."))}', img[:,:min_x])
  cv2.imwrite(f'{BACKGROUNDS_PATH}/{"_3.".join(pic.split("."))}', img[:,max_x:])

output_file.close()
