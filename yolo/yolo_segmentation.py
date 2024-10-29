from PIL import Image, ImageDraw
import numpy as np
from ultralytics import YOLO
import math, csv, random, cv2, os

def read_robot_traj(csv_file):
    robot_path = []
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        next(reader)
        for row in reader:

            robot_path.append(
                {
                     't': float(row[0]),
                     'x': float(row[1]),
                     'y': float(row[2]),
                     'yaw': float(row[3]),
                     'depth_image_path': row[4],
                     'clr_image_path': row[5],
                }
            )
    return robot_path

def get_isolated_classes(model, color_image_path):
    # Yolov11 segmentation tutorial: https://docs.ultralytics.com/guides/isolating-segmentation-objects/
    
    result = model(color_image_path, verbose=False)[0]
    isolated_images = []

    for i, class_idx in enumerate(result.boxes.cls.tolist()):
        label = result.names[class_idx]
        conf = result.boxes.conf[i]

        detectable_objects = [
            'box', 
            'open_box',
            'basket',
            'pallet_unloaded',
            'pallet_loaded',
            'forklift',
            'person',
            'backpack'
        ]

        if label in detectable_objects and conf >= 0.75:
            # Copy the original image
            img = np.copy(result.orig_img)

            # Create a mask for the current object
            b_mask = np.zeros(img.shape[:2], np.uint8)
            contour = result.masks.xy[i]
            contour = contour.astype(np.int32)
            contour = contour.reshape(-1, 1, 2)
            _ = cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED)

            # Stack image and mask to make the background transparent
            isolated = np.dstack([img, b_mask])
            
            # Convert isolated array to Image for text overlay
            isolated_image = Image.fromarray(isolated.astype('uint8'))

            # Add the label text to the image using OpenCV
            x, y = int(result.boxes.xyxy[i][0]), int(result.boxes.xyxy[i][1])

            # Set padding and adjust the y position if it's too close to the top of the image
            padding = 10
            if y - padding < 0:
                y += padding  # Move text inside the top boundary

            # Calculate the width and height of the text box
            (text_width, text_height), _ = cv2.getTextSize(f"{label} ({conf:.2f})", cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)

            # Adjust x position if text goes beyond the right boundary of the image
            if x + text_width > isolated.shape[1]:
                x = isolated.shape[1] - text_width - padding

            # Place text on the image
            img_with_label = cv2.putText(
                isolated,
                f"{label} ({conf:.2f})",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,  # Font size
                (0, 255, 0, 255),  # Green text in RGBA
                2,  # Thickness
                cv2.LINE_AA
            )

            # Convert back to PIL Image and add to the list
            isolated_images.append(Image.fromarray(img_with_label.astype('uint8')))

    return isolated_images, Image.fromarray(result.orig_img.astype('uint8'))

def get_isolated_image_realworld_xyz(im, depth_image_path):

    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

    real_x_dist_samples = []
    real_y_dist_samples = []
    real_z_dist_samples = []

    image_xs = []
    image_ys = []

    fx = 76.43678283691406
    fy = 76.43678283691406
    cx = 87.5
    cy = 50.0

    draw = ImageDraw.Draw(im)

    while len(image_xs) < 1000:
        x_img = random.randint(0, im.size[0] - 1)
        y_img = random.randint(0, im.size[1] - 1)

        _, _, _, a = im.getpixel((x_img, y_img))

        if a != 0 and not math.isinf(depth_image[y_img, x_img]):

            image_xs.append(x_img)
            image_ys.append(y_img)   

            x_real_world = (x_img - cx) * depth_image[y_img, x_img] / fx
            y_real_world = (y_img - cy) * depth_image[y_img, x_img] / fy

            real_x_dist_samples.append(x_real_world) 
            real_y_dist_samples.append(y_real_world) 
            real_z_dist_samples.append(depth_image[y_img, x_img]) 

            draw.point([x_img, y_img], fill='orange')

    avg_real_x_dist = sum(real_x_dist_samples) / len(real_x_dist_samples)
    avg_real_y_dist = sum(real_y_dist_samples) / len(real_y_dist_samples)
    avg_real_z_dist = sum(real_z_dist_samples) / len(real_z_dist_samples)

    return avg_real_x_dist, avg_real_y_dist, avg_real_z_dist, im


def transform_obs_to_map_frame(cam_to_obj_x, cam_to_obj_y, cam_to_obj_z):
    base_link_to_front_cam = np.matrix([
        [0.0, 0.0, 1.0, 0.41],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])

    base_link_to_left_cam = np.matrix([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.247],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])

    base_link_to_rear_cam = np.matrix([
        [0.0, 0.0, -1.0, -0.41],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0 , 0.0, 1.0],
    ])

    base_link_to_right_cam = np.matrix([
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, -0.247],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])

    obj_in_cam = np.matrix([
        [cam_to_obj_x],
        [cam_to_obj_y],
        [cam_to_obj_z],
        [1.0]
    ])

    map_to_robot_x = slam_moment['x']
    map_to_robot_y = slam_moment['y']
    map_to_robot_yaw = slam_moment['yaw']

    c = math.cos(map_to_robot_yaw)
    s = math.sin(map_to_robot_yaw)

    map_to_base_link = np.matrix([
        [c, -s, 0.0, map_to_robot_x],
        [s, c, 0.0,  map_to_robot_y],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])

    if cam == 'front':

        obj_in_robot_frame = base_link_to_front_cam * obj_in_cam

    elif cam == 'left':

        obj_in_robot_frame = base_link_to_left_cam * obj_in_cam

    elif cam == 'rear':

        obj_in_robot_frame = base_link_to_rear_cam * obj_in_cam

    elif cam == 'right':

        obj_in_robot_frame = base_link_to_right_cam * obj_in_cam

    obj_in_map_frame = map_to_base_link * obj_in_robot_frame

    x_map_to_obj = obj_in_map_frame[0,0]
    y_map_to_obj = obj_in_map_frame[1,0]

    obs_coord = [
        x_map_to_obj,
        y_map_to_obj,
    ]

    return obs_coord

datasets_dir = '/home/yolo/datasets/'
dir_names = [d for d in os.listdir(datasets_dir) if os.path.isdir(os.path.join(datasets_dir, d))]
numeric_dirs = [int(d) for d in dir_names if d.isdigit()]
dataset_dir = str(max(numeric_dirs)) if numeric_dirs else str(0)
raw_data_dir = datasets_dir + dataset_dir + '/raw/'

slam_moments = read_robot_traj(raw_data_dir + 'slam_moments.csv')

obstacles_coords = []

model = YOLO("./runs/segment/train/weights/best.pt")
j = 0

for slam_moment in slam_moments:    

    cam = slam_moment['clr_image_path'].split('/')[6].split('_')[0]

    isolated_images, orig_image = get_isolated_classes(model, slam_moment['clr_image_path'])

    background_image = orig_image.copy()

    print(j, '/', len(slam_moments))
    j += 1

    for i, isolated_image in enumerate(isolated_images):

        cam_to_obj_x, cam_to_obj_y, cam_to_obj_z, im = get_isolated_image_realworld_xyz(isolated_image, slam_moment['depth_image_path'])

        iso_clr_image_filename = datasets_dir + dataset_dir + "/isolated/" + cam + "/" + str(slam_moment['t']) + "_" + str(i) + ".png"
        os.makedirs(os.path.dirname(iso_clr_image_filename), exist_ok=True)
        im.save(iso_clr_image_filename)        

        obs_coord = transform_obs_to_map_frame(cam_to_obj_x, cam_to_obj_y, cam_to_obj_z)      
        obs_coord.insert(0, slam_moment['t'])
        obs_coord.append(iso_clr_image_filename)

        obstacles_coords.append(obs_coord)

        background_image.paste(isolated_image, (0, 0), isolated_image)

    overlay_path = datasets_dir + dataset_dir + "/overlayed/" + cam + "/" + str(slam_moment['t']) + "_" + str(i) + ".png"
    os.makedirs(os.path.dirname(overlay_path), exist_ok=True)
    background_image.save(overlay_path)



save_file = datasets_dir + dataset_dir + '/obstacle_coords.csv'
with open(save_file, 'w') as file:
    writer = csv.writer(file)

    writer.writerows(obstacles_coords)



