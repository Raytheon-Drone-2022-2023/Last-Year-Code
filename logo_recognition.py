import numpy as np

def detect_white_and_red(img: np.ndarray, is_white=lambda x: False, is_red=lambda x: False):
  '''
  This function is used to filter out "white" and "red" pixels within an image
  The image is expected to contain pixel values in the range of 0 - 255, inclusive
  The image is also expected to be of dimensions "width x height x channel" or "height x width x channel".
  White pixels are red pixels are defined by lambda functions which are passed as arguments to the function

  Returns:
    2 numpy arrays, each of dimensions "width x height" or "height x width"
    The first is a 2D array of whether the pixels qualify as white
    The second is a 2D array of whether the pixels qualify as red

  Lambda functions' return values will be treated as booleans, so they can return boolean values, but if you
    return a float or an integer other than 0 or 1 then this function may not function as intended

  Example lambda function:
    is_white = lambda x: x[0] > 0.8 * x[1] > 0.8 * x[2] > 0.8
    In this case, the lambda function of ([0.9, 0.85, 0.7]) returns:
      0.9 > 0.8  *  0.85 > 0.8  *  0.7 > 0.8
      True       *  True        *  False
      True                      *  False
      False
  '''

  img = img / 255. # Normalize
  white_pixels = is_white(img)
  red_pixels = is_red(img)

  return white_pixels, red_pixels

def detect_logo(img: np.ndarray, is_white=lambda x: False, is_red=lambda x: False, alpha=0, expected_sizes=[50], granularity=0.6, top_regions=5, fine_tuning_passes=10):
  '''
  This function returns the x, y, w, h of the suspected logo, or None if no logo is detected
  (x, y) is the top-left corner of the bounding box

  Paramters:
    img: WxHxC or HxWxC image with pixel values between 0 and 255 (inclusive)
    is_white: See is_white entry for detect_white_and_red
    is_red: See is_red entry for detect_white_and_red
    alpha: How much to favor having a lot of white and red versus having the centers of the colors nearby
            Range: 0 - 1
            Closer to 0 favors the white-red heursitic while closer to 1 favors the near centers heurisitc
    expected_size: List of expected sizes (default 50x50) that the logo could appear at
    granularity: Hyperparameter that controls how far across the 50x50 window to move for each check (default 50 * 0.6 = 30)
    top_regions: Hyperparameter that controls the number of regions to check for a potential match before the fine-tuning process
    fine_tuning_passes: Hyperparameter that controls the number of times to fine-tune the logo bounding box. Generally the more passes, the more the logo shrinks
  '''

  # Get a list of all the red and white pixels
  white_pixels, red_pixels = detect_white_and_red(img, is_white, is_red)

  # A list of regions to keep track of all the possible logos
  combined_regions = []

  for expected_size in expected_sizes: # For all expected sizes
    for y in range(0, img.shape[0] - expected_size, int(expected_size * granularity)): # For each row in the image
      for x in range(0, img.shape[1] - expected_size, int(expected_size * granularity)): # For each column in the image
        percent_white = np.sum(white_pixels[y:y+expected_size, x:x+expected_size]) / expected_size / expected_size # Find how much of the region is white
        percent_red = np.sum(red_pixels[y:y+expected_size, x:x+expected_size]) / expected_size / expected_size # Find how much of the region is red

        num_white = np.sum(white_pixels[y:y+expected_size, x:x+expected_size])
        num_red = np.sum(red_pixels[y:y+expected_size, x:x+expected_size])

        if num_white == 0 or num_red == 0:
          continue



        white_count_row = np.sum(white_pixels[y:y+expected_size, x:x+expected_size], axis=0)
        white_center_row = y + np.sum(np.array(range(expected_size)) * white_count_row) / num_white

        white_count_col = np.sum(white_pixels[y:y+expected_size, x:x+expected_size], axis=1)
        white_center_col = x + np.sum(np.array(range(expected_size)) * white_count_col) / num_white
        
        red_count_row = np.sum(red_pixels[y:y+expected_size, x:x+expected_size], axis=0)
        red_center_row = y + np.sum(np.array(range(expected_size)) * red_count_row) / num_red

        red_count_col = np.sum(red_pixels[y:y+expected_size, x:x+expected_size], axis=1)
        red_center_col = x + np.sum(np.array(range(expected_size)) * red_count_col) / num_red

        # TODO Explain heurisitics
        white_red_heuristic = percent_red * percent_white * 100
        center_match_heuristic = ((white_center_col - red_center_col)**2 + (white_center_row - red_center_row)**2)**0.5
        
        # Convert to maximization problem
        center_match_heuristic = max(center_match_heuristic, 1) # Make all numbers >= 1
        center_match_heuristic = 1/center_match_heuristic * 100

        score = alpha * center_match_heuristic + (1-alpha) * center_match_heuristic

        combined_regions.append({'x': x, 'y': y, 'size': expected_size,
                                  'white_red_heuristic': white_red_heuristic,
                                  'center_match_heuristic': center_match_heuristic,
                                  'center_white': (white_center_col, white_center_row),
                                  'center_red': (red_center_col, red_center_row),
                                  'score': score})
  
  # If there are no matches then just return 'None'
  if len(combined_regions) == 0:
    print('Couldn\'t find any combined regions!')
    return None

  # Sort the regions by the aforementioned 'combined' field
  combined_regions = sorted(combined_regions, key=lambda x: x['score'], reverse=True)

  # Enlarge the top regions to capture the entire logo, in case the logo landed on a corner of one of the regions that were checked
  large_combined_regions = []
  for i in range(min(len(combined_regions), top_regions)):
    expected_size = combined_regions[i]['size']
    half_gran = int(granularity * expected_size / 2) # Get the value that is half of how far the checked region moves per iteration
    x, y = int(combined_regions[i]['x'] - half_gran), int(combined_regions[i]['y'] - half_gran) # Get the values of moving halfway back in x and y directions
    large_size = int(expected_size * (1 + granularity)) # Increase the size to be more likely to include the entire logo


    # Bounds clipping
    max_y = min(y+large_size, img.shape[0])
    max_x = min(x+large_size, img.shape[1])

    y = max(y, 0)
    x = max(x, 0)

    # Recalculate the heurisitic used for determining the most likely logo
    percent_white = np.sum(white_pixels[y:max_y, x:max_x]) / (max_y - y) / (max_x - x)
    percent_red = np.sum(red_pixels[y:max_y, x:x+max_x]) / (max_y - y) / (max_x - x)

    num_white = np.sum(white_pixels[y:y+expected_size, x:x+expected_size])
    num_red = np.sum(red_pixels[y:y+expected_size, x:x+expected_size])

    if num_white == 0 or num_red == 0:
      continue

    white_count_row = np.sum(white_pixels[y:y+expected_size, x:x+expected_size], axis=0)
    white_center_row = y + np.sum(np.array(range(expected_size)) * white_count_row) / num_white

    white_count_col = np.sum(white_pixels[y:y+expected_size, x:x+expected_size], axis=1)
    white_center_col = x + np.sum(np.array(range(expected_size)) * white_count_col) / num_white
    
    red_count_row = np.sum(red_pixels[y:y+expected_size, x:x+expected_size], axis=0)
    red_center_row = y + np.sum(np.array(range(expected_size)) * red_count_row) / num_red

    red_count_col = np.sum(red_pixels[y:y+expected_size, x:x+expected_size], axis=1)
    red_center_col = x + np.sum(np.array(range(expected_size)) * red_count_col) / num_red

    # The white/red heuristic measures how many white and red pixels are in the given section
    # These values are multiplied by each other to prioritize sections with some white and some red compared to a lot of white and very little red
    
    # The center match heurisitc measures how close the centers of the red and white pixels are
    # The closer the pixels, the higher the score

    #
    white_red_heuristic = percent_red * percent_white * 100
    center_match_heuristic = ((white_center_col - red_center_col)**2 + (white_center_row - red_center_row)**2)**0.5
    
    # Convert to maximization problem
    center_match_heuristic = max(center_match_heuristic, 1) # Make all numbers >= 1
    center_match_heuristic = 1/center_match_heuristic * 100

    score = alpha * center_match_heuristic + (1-alpha) * center_match_heuristic

    large_combined_regions.append({'x': x, 'y': y, 'max_x': max_x, 'max_y': max_y, 'size': expected_size,
                              'white_red_heuristic': white_red_heuristic,
                              'center_match_heuristic': center_match_heuristic,
                              'center_white': (white_center_col, white_center_row),
                              'center_red': (red_center_col, red_center_row),
                              'score': score})
  
  # Get the most likely logo using the same 'combined' heuristic
  large_combined_regions = sorted(large_combined_regions, key=lambda x: x['score'], reverse=True)
  most_likely_region = large_combined_regions[0]

  # Trim away parts of the logo that are not actually part of the logo
  # More iterations mean more fine-tuning which means less of the logo counts
  for i in range(fine_tuning_passes):
    # Count the number of white + red pixels across each row, essentially the width of the logo at that row
    widths = []
    for row in range(min(most_likely_region['max_x'], img.shape[1]) - most_likely_region['x']):
      white_sum = np.sum(white_pixels.T[most_likely_region['x'] + row])
      red_sum = np.sum(red_pixels.T[most_likely_region['x'] + row])
      widths.append(white_sum + red_sum)

    # Count the number of white + red pixels across each column, essentially the height of the logo at that column
    heights = []
    for col in range(min(most_likely_region['max_y'], img.shape[0]) - most_likely_region['y']):
      white_sum = np.sum(white_pixels[most_likely_region['y'] + col]) 
      red_sum = np.sum(red_pixels[most_likely_region['y'] + col])
      heights.append(white_sum + red_sum)

    # Get the max width and height
    try:
      max_width = max(widths)
      max_height = max(heights)
    except Exception as ex:
      # Sometimes this fails? In which case return None
      return None

    # Move the edges in until you reach half the width of the logo
    min_x, min_y = 0, 0
    while widths[min_x] < max_width * 0.5: min_x += 1
    while heights[min_y] < max_height * 0.5: min_y += 1

    w, h = len(widths)-1, len(heights)-1
    while widths[w] < max_width * 0.5: w -= 1
    while heights[h] < max_height * 0.5: h -= 1

    # Check to see if finished fine-tuning
    if min_x == 0 and min_y == 0 and w == len(widths) - 1 and h == len(heights) - 1:
      break

    most_likely_region['x'] += min_x
    most_likely_region['y'] += min_y
    most_likely_region['max_x'] = most_likely_region['x'] + w
    most_likely_region['max_y'] = most_likely_region['y'] + h
    

  x = min_x + most_likely_region['x']
  y = min_y + most_likely_region['y']


  white_count_row = np.sum(white_pixels[y:y+expected_size, x:x+expected_size], axis=0)
  white_center_row = y + np.sum(np.array(range(expected_size)) * white_count_row) / num_white

  white_count_col = np.sum(white_pixels[y:y+expected_size, x:x+expected_size], axis=1)
  white_center_col = x + np.sum(np.array(range(expected_size)) * white_count_col) / num_white
  
  red_count_row = np.sum(red_pixels[y:y+expected_size, x:x+expected_size], axis=0)
  red_center_row = y + np.sum(np.array(range(expected_size)) * red_count_row) / num_red

  red_count_col = np.sum(red_pixels[y:y+expected_size, x:x+expected_size], axis=1)
  red_center_col = x + np.sum(np.array(range(expected_size)) * red_count_col) / num_red

  return x, y, w, h