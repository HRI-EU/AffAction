import os
import numpy as np
from PIL import Image

# Path to the parent folder containing all rollout folders
parent_directory = r'C:\Users\RE900373\Documents\Trainingdata'

def process_rollout_folder(rollout_folder, num_steps):
    data_file = os.path.join(rollout_folder, 'data.txt')
    
    # Initialize lists to store image arrays and coordinates
    image_arrays = []
    coordinates = []
    
    # Read the data.txt file
    with open(data_file, 'r') as f:
        lines = f.readlines()
    
    # Process each line and gather `num_steps` consecutive rows
    for i in range(len(lines) - num_steps + 1):  # Ensure there are enough rows
        # Parse the current line
        parts = lines[i].split()
        img_filename = parts[1]
        
        # Load the image file and convert it to a numpy array
        img_path = os.path.join(rollout_folder, img_filename)
        img = Image.open(img_path)
        img_array = np.array(img)
        
        # Collect the image array
        image_arrays.append(img_array)
        
        # Gather the next `num_steps` rows of coordinates
        consecutive_coordinates = []
        for j in range(num_steps):
            coord_parts = list(map(float, lines[i + j].split()[2:]))
            consecutive_coordinates.append(coord_parts)
        
        # Append the `num_steps` rows of coordinates for this image
        coordinates.append(consecutive_coordinates)
    
    # Convert lists to numpy arrays
    image_arrays = np.array(image_arrays)
    coordinates = np.array(coordinates)
    
    # Save to npz file
    npz_filename = os.path.join(rollout_folder, 'processed_data.npz')
    np.savez(npz_filename, array1=image_arrays, array2=coordinates)
    print(f"Saved: {npz_filename}")

def process_all_rollouts(parent_directory, num_steps):
    for folder in os.listdir(parent_directory):
        rollout_folder = os.path.join(parent_directory, folder)
        if os.path.isdir(rollout_folder):
            print(f"Processing folder: {rollout_folder}")
            process_rollout_folder(rollout_folder, num_steps)

def combine_npz_files(parent_directory, output_filename='combined_data.npz'):
    # Initialize lists to store combined data
    all_images = []
    all_coordinates = []
    
    # Traverse all subdirectories
    for root, dirs, files in os.walk(parent_directory):
        for file in files:
            if file.endswith('.npz'):
                # Load the npz file
                npz_path = os.path.join(root, file)
                data = np.load(npz_path)
                
                # Get array1 (images) and array2 (coordinates)
                images = data['array1']
                coordinates = data['array2']
                
                # Only process files with at least 16 images
                if images.shape[0] >= 16 and coordinates.shape[0] >= 16:
                    all_images.append(images)
                    all_coordinates.append(coordinates)
                else:
                    print(f"Skipping file {file} - contains fewer than 16 images or coordinates.")
    
    # Check if there are any valid images and coordinates to concatenate
    if all_images and all_coordinates:
        # Concatenate all images and coordinates into single numpy arrays
        combined_images = np.concatenate(all_images, axis=0)
        combined_coordinates = np.concatenate(all_coordinates, axis=0)
    
        # Save the combined data into a single npz file
        np.savez(output_filename, array1=combined_images, array2=combined_coordinates)
        print(f"Combined data saved as: {output_filename}")
    else:
        print("No valid npz files found with at least 16 images.")


# Call the function to process all rollouts
num_steps = 16  # Set the number of steps here
process_all_rollouts(parent_directory, num_steps)

# Call the function to combine all npz files in the parent directory
combine_npz_files(parent_directory, 'combined_data.npz')

data = np.load("combined_data.npz")

print("Images shape: ", data['array1'].shape)
print("Policy shape: ", data['array2'].shape)

# Convert the NumPy array to a PIL Image and show it
image_array = data['array1'][0]
image = Image.fromarray(image_array)
image.show()

# Show the policy
np.set_printoptions(precision=8, suppress=True, linewidth=100)
print(data['array2'][0])

# Slicing to retain only the first 2 elements in the last dimension
policy2d = data['array2'][:, :, :2]

# Print the shape to verify
print("New array shape:", policy2d.shape)

# Save both arrays to an .npz file
np.savez('output_2d.npz', array1=data['array1'], array2=policy2d)

# Or use compressed version
np.savez_compressed('output_2d_compressed.npz', array1=data['array1'], array2=policy2d)