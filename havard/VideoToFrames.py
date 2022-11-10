from sys import argv, stdout
import cv2

def main(filename):
  video_file = None
  try:
    video_file = cv2.VideoCapture(argv[1])
  except ex:
    print(ex)
    print('Problem opening input stream')
    return

  if not video_file.isOpened():
    print('Capture stream is not open')
    return

  num_frames = int(video_file.get(cv2.CAP_PROP_FRAME_COUNT))
  fps = video_file.get(cv2.CAP_PROP_FPS)
  print(f'Found {num_frames} frames @ {fps} FPS')

  found, frame = video_file.read()
  for i in range(num_frames):
    stdout.write(f'\u001b[100D{i+1}/{num_frames}')
    stdout.flush()
    cv2.imwrite(f'data/output/frame_{i}.jpg', frame)
    found, frame = video_file.read()
    if not found: break

if __name__ == '__main__':
  input_file = argv[1] if len(argv) > 1 else input('Input file: ')
  main(input_file)
