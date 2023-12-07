import numpy as np
import os
import cv2
# import torch
# from scipy.ndimage import convolve

from .rgb import *


def closest_colours(requested_colour, colours):
    '''
    Find the closest colours to the requested colour

    This returns the closest three matches
    '''

    distances = {color: np.linalg.norm(
        np.array(rgb_val) - requested_colour) for color, rgb_val in colours.items()}
    sorted_colors = sorted(distances.items(), key=lambda x: x[1])
    top_three_colors = sorted_colors[:3]
    formatted_colors = [(color_name, distance)
                        for color_name, distance in top_three_colors]

    return formatted_colors


# def avg_color_float(rgb_image: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
#     mask = mask.bool()
#     avg_colors = torch.zeros((rgb_image.size(0), mask.size(1), rgb_image.size(1)), device=rgb_image.device)
#     for i in range(rgb_image.size(0)):
#         for j in range(mask.size(1)):
#             for k in range(rgb_image.size(1)):
#                 valid_pixels = torch.masked_select(rgb_image[i, k], mask[i, j])
#                 avg_color = valid_pixels.float().mean() if valid_pixels.numel() > 0 else torch.tensor(0.0)
#                 avg_colors[i, j, k] = avg_color

#     return avg_colors  # / 255.0


# def median_color_float(rgb_image: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
#     mask = mask.bool()
#     median_colors = torch.zeros((rgb_image.size(0), mask.size(1), rgb_image.size(1)), device=rgb_image.device)
#     for i in range(rgb_image.size(0)):
#         for j in range(mask.size(1)):
#             for k in range(rgb_image.size(1)):
#                 valid_pixels = torch.masked_select(rgb_image[i, k], mask[i, j])
#                 if valid_pixels.numel() > 0:
#                     median_value = valid_pixels.median()
#                 else:
#                     median_value = torch.tensor(0.0, device=rgb_image.device)
#                 median_colors[i, j, k] = median_value
#     return median_colors  # / 255.0


# def plot_with_matplotlib(frame, categories, masks, predictions, colours):
#     """Generate an image with matplotlib, showing the original frame and masks with titles and color overlays."""
#     assert len(masks) == len(categories) == len(predictions), "Length of masks, categories, and predictions must match."
    
#     num_masks = len(masks)
#     cols = 3
#     rows = (num_masks + 1) // cols + ((num_masks + 1) % cols > 0)  # Adding 1 for the frame
#     position = range(1, num_masks + 2)  # +2 to include the frame in the count

#     fig = plt.figure(figsize=(15, rows * 3))  # Adjust the size as needed
    
#     # Add the frame as the first image
#     ax = fig.add_subplot(rows, cols, 1)
#     # frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#     ax.imshow(frame)
#     ax.set_title('Original Frame')
#     ax.axis('off')
    
#     # Iterate over the masks
#     for i, idx in enumerate(position[1:], start=1):  # Skip 1 for the frame
#         ax = fig.add_subplot(rows, cols, idx)
        
#         # Create an RGB image for the colored mask
#         colored_mask = np.stack([masks[i-1]]*3, axis=-1)  # i-1 because we skip the frame in position
        
#         # Apply color if category is detected and color is provided
#         if predictions[i-1]:
#             if (i-1) < len(colours):
#                 color = np.array(colours[i-1], dtype=np.uint8)  # Convert color to uint8
#                 color_mask = np.zeros_like(colored_mask)  # Initialize color_mask with the same shape as colored_mask
#                 color_mask[..., 0] = masks[i-1] * color[0]  # Apply color channel 0
#                 color_mask[..., 1] = masks[i-1] * color[1]  # Apply color channel 1
#                 color_mask[..., 2] = masks[i-1] * color[2]  # Apply color channel 2
#                 # Now combine the colored mask with the original grayscale mask
#                 colored_mask = np.where(masks[i-1][:, :, None], color_mask, colored_mask).astype(np.uint8)
#                 # Show the colored mask
#                 ax.imshow(colored_mask)
#                 # print(np.max(mask_image))
#                 # mask_image = masks[i-1]
#                 # ax.imshow(mask_image, cmap="gray")
#             else:
#                 # If there's no color provided for this category, use white color
#                 mask_image = masks[i-1]
#                 ax.imshow(mask_image, cmap="gray")
#         else:
#             # If the category is not detected, keep the mask black
#             mask_image = masks[i-1]
#             ax.imshow(mask_image, cmap="gray")


#         # mask_image = masks[i-1]
#         # ax.imshow(mask_image, cmap="gray")
        
#         # Set title with the detection status
#         detection_status = 'yes' if predictions[i-1] else 'no'
#         ax.set_title(f"{categories[i-1]} - {detection_status}")
#         ax.axis('off')
    
#     plt.tight_layout()
#     fig.canvas.draw()
    
#     # Retrieve buffer and close the plot to avoid memory issues
#     data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
#     data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
#     plt.close(fig)
    
#     return data


def count_colours_in_masked_area(img, mask, colours, filter_size=3, sort=False):
    """
    Counts the number of pixels of each color within the masked area of an image.

    Parameters:
    img (numpy.ndarray): An RGB image, with the shape (height, width, 3).
    mask (numpy.ndarray): A binary mask, with the shape (height, width), where 1 indicates the area of interest.
    colours (dict): A dictionary where keys are color names and values are the corresponding RGB values.
    filter_size (int): The size of the convolution filter to apply for smoothing the image, default is 3.
    sort (bool): Whether to return a sorted list of colors based on pixel count, default is False.

    Returns:
    dict: A dictionary containing the count of pixels for each color in the masked area.
    If sort is True, it also returns a list of tuples, each containing a color name, its proportion in the masked area, and the pixel count. This list is sorted in descending order based on pixel count.

    The function first applies an averaging filter to the image for smoothing. Then, it calculates the Euclidean distance of each pixel in the masked area to the predefined colors. It identifies the closest color for each pixel, counts the occurrences of each color, and creates a dictionary mapping colors to their respective counts. If sorting is requested, it also calculates the proportion of each color and returns a sorted list of colors based on their pixel count.
    """
    avg_filter = np.ones((filter_size, filter_size, 3)) / (filter_size ** 2)
    img_filtered = img
    # img_filtered = convolve(img, avg_filter, mode='constant', cval=0.0)
    colours_array = np.array(list(colours.values()))
    masked_img = img_filtered[mask == 1]
    distances = np.linalg.norm(masked_img[:, None] - colours_array, axis=2)
    closest_colours = np.argmin(distances, axis=1)
    unique, counts = np.unique(closest_colours, return_counts=True)
    colour_counts = {list(colours.keys())[i]: count for i, count in zip(unique, counts)}
    if sort:
        total_pixels = sum(counts)
        sorted_colours = sorted(((list(colours.keys())[i], count / total_pixels, count) 
                                 for i, count in zip(unique, counts)), key=lambda item: item[2], reverse=True)
        return colour_counts, sorted_colours

    return colour_counts


def average_colours_by_label(labels, colours):
    """
    Computes the average values of colours associated with each label.

    Parameters:
    labels (dict): A dictionary where keys are label names and values are lists of binary values (0 or 1). Each list represents whether a certain feature (labelled by the key) is present (1) or not (0) in a set of instances.
    colours (dict): A dictionary where keys are label names and values are dictionaries. Each inner dictionary maps colour names to lists of values (e.g., pixel counts or intensities) associated with that colour for each instance.

    Returns:
    dict: A dictionary where keys are label names and values are sorted lists of tuples. Each tuple contains a colour name and its average value calculated only from instances where the label is present (1). The tuples are sorted by average values in descending order.

    The function iterates through each label, calculating the average value for each colour only from instances where the label value is 1 (present). It then sorts these average values in descending order for each label and returns this sorted list along with the label name in a dictionary.
    """
    averaged_colours = {}

    for label, label_values in labels.items():
        if label not in colours.keys():
            continue

        colour_values = colours[label]
        averages = {}

        for colour, values in colour_values.items():
            valid_values = [value for value, label_value in zip(values, label_values) if label_value == 1]
            if valid_values:
                averages[colour] = sum(valid_values) / len(valid_values)

        sorted_colours = sorted(averages.items(), key=lambda item: item[1], reverse=True)
        averaged_colours[label] = sorted_colours

    return averaged_colours


def load_images_to_dict(root_dir):
    """
    Load images from a specified directory into a dictionary, removing file extensions from the keys.

    Parameters:
    root_dir (str): The root directory containing the images.

    Returns:
    dict: A dictionary with image names (without extensions) as keys and their corresponding numpy arrays as values.
    """
    image_dict = {}
    for filename in os.listdir(root_dir):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
            img_path = os.path.join(root_dir, filename)
            # Read the image using OpenCV
            img = cv2.imread(img_path)
            # Convert it from BGR to RGB color space
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # Remove the file extension from the filename
            name_without_extension = os.path.splitext(filename)[0]
            image_dict[name_without_extension] = img

    return image_dict


def generate_colour_table(image_dict: dict, colour_map: dict):
    """
    Generates a colour table for each image in the given dictionary, counting the colours in each image.

    Parameters:
    image_dict (dict): A dictionary where keys are image identifiers and values are image arrays in the format (height, width, 3).
    colour_map (dict): A dictionary mapping colour names to their respective RGB values.

    Returns:
    dict: A dictionary where keys are image identifiers and values are colour tables. Each colour table is generated by the 'count_colours_in_masked_area' function and contains a count of how many times each colour (as defined in colour_map) appears in the corresponding image.

    For each image in the image_dict, this function creates a mask that covers the entire image and uses 'count_colours_in_masked_area' to count the occurrences of each colour in the colour_map within the image. The results are stored in a new dictionary, mapping each image identifier to its corresponding colour table.
    """
    colour_table = {}
    for k in image_dict.keys():
        colour_table[k] = count_colours_in_masked_area(image_dict[k], np.ones((image_dict[k].shape[0], image_dict[k].shape[1])), colour_map, sort=True)
    return colour_table


def compare_colour_distributions(averaged_colours_list, colour_table_dict):
    """
    Compares colour distributions between an averaged colours list and a dictionary of colour tables by calculating the Euclidean distance.

    Parameters:
    averaged_colours_list (list): A list of tuples, where each tuple contains a colour name and its proportion. This is typically the output from 'average_colours_by_label'.
    colour_table_dict (dict): A dictionary where keys are image identifiers and values are colour tables. Each colour table is a list of tuples, each containing a colour name, its proportion in the image, and the pixel count.

    Returns:
    dict: A dictionary where keys are image identifiers and values are the Euclidean distances between the colour distribution in the image and the averaged_colours_list.

    The function iterates over each image's colour table in colour_table_dict. For each image, it calculates the Euclidean distance between the colour proportions in averaged_colours_list and the colour proportions in the image's colour table. The results are stored in a dictionary, mapping each image identifier to the calculated distance.
    """
    distances = {}

    avg_colours_dict = {colour: proportion for colour, proportion in averaged_colours_list}

    for image_name, colour_data in colour_table_dict.items():
        colour_proportions = {colour: proportion for colour, proportion, _ in colour_data[1]}

        common_colours = set(avg_colours_dict.keys()) & set(colour_proportions.keys())
        avg_values = [avg_colours_dict.get(colour, 0) for colour in common_colours]
        prop_values = [colour_proportions.get(colour, 0) for colour in common_colours]

        distances[image_name] = np.linalg.norm(np.array(avg_values) - np.array(prop_values))

    sorted_distances = sorted(distances.items(), key=lambda item: item[1])

    return sorted_distances

# Example usage
# sorted_distances = compare_colour_distributions(averaged_colours, colour_table)

def extract_top_colours_by_threshold(colour_list, threshold):
    """
    Extracts top colours based on a cumulative proportion threshold.

    Parameters:
    colour_list (list): A list of tuples, each being a 2-element (colour, proportion) or 
                        a 3-element (colour, proportion, count) tuple.
    threshold (float): A float between 0 and 1, representing the threshold for the cumulative proportion.

    Returns:
    list: A list of tuples (colour, proportion), sorted by proportion in descending order, 
          whose cumulative proportion just exceeds the threshold.
    """
    # Sort the list by proportion in descending order
    sorted_colours = sorted(colour_list, key=lambda x: x[1], reverse=True)

    # Extract top colours based on the cumulative proportion threshold
    cumulative_proportion = 0.0
    top_colours = []
    for colour in sorted_colours:
        cumulative_proportion += colour[1]
        top_colours.append((colour[0], colour[1]))
        if cumulative_proportion >= threshold:
            break

    return top_colours


def find_nearest_colour_family(colour, colour_families):
    """
    Determines the nearest colour family for a given colour.

    Parameters:
    colour (tuple): The colour in RGB format.
    colour_families (dict): A dictionary where keys are family names and values are lists of representative RGB colours for each family.

    Returns:
    str: The name of the nearest colour family.
    """
    min_distance = float('inf')
    nearest_family = None

    for family, representative_colours in colour_families.items():
        for rep_colour in representative_colours:
            distance = np.linalg.norm(np.array(colour) - np.array(rep_colour))
            if distance < min_distance:
                min_distance = distance
                nearest_family = family

    return nearest_family


def find_nearest_colour_family(colour, colour_families):
    """
    Determines the nearest colour family for a given colour based on the minimum Euclidean distance.

    Parameters:
    colour (tuple): The colour in RGB format.
    colour_families (dict): A dictionary where keys are family names and values are lists of representative RGB colours for each family.

    Returns:
    str: The name of the nearest colour family.
    """
    min_distance = float('inf')
    nearest_family = None

    # Convert colour to numpy array for distance calculation
    colour = np.array(colour)

    for family, family_colours in colour_families.items():
        for rep_colour in family_colours:
            # Calculate the Euclidean distance
            distance = np.linalg.norm(colour - np.array(rep_colour))
            if distance < min_distance:
                min_distance = distance
                nearest_family = family

    return nearest_family

