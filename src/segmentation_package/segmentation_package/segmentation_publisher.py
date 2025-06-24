import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import cv2
from std_msgs.msg import Float32, Int32MultiArray, String, Int32, Float32MultiArray
#from python_nodes_interfaces.msg import BoolStamped, Int32Stamped, Float32MultiArrayStamped#, StringMultiArrayStamped
from geometry_msgs.msg import Twist, TwistStamped
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import os
import time
import sys

#TODO
# # Path to your virtual environment
# venv_path = "/home/orin/torch_env"
venv_path = "/workspaces/gazebo_ws/sh_venv"

# # Manually set environment variables to activate the venv
os.environ["VIRTUAL_ENV"] = venv_path
os.environ["PATH"] = f"{venv_path}/bin:" + os.environ["PATH"]


# # Add the virtual environment's site-packages to sys.path
# site_packages = os.path.join(venv_path, "lib", "python3.10", "site-packages")  # Adjust Python version
# sys.path.insert(0, site_packages)
site_packages = os.path.join(venv_path, "lib", "python3.12", "site-packages")  # Adjust Python version
sys.path.insert(0, site_packages)

# import files
from PIL import Image
import torch
import torchvision.transforms as T
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor
import yaml

bridge = CvBridge()
#driving_mode node 0: manual 1: GPS 2: lane following 3: pullin 4: pullout 5: roundabout
#TODO
Object_Path = os.path.dirname(os.path.abspath(__file__))
Model_Path = "/home/orin/kingston/Documents/NN_Models_and_Scripts"
PACKAGE_DIR = get_package_share_directory('segmentation_package')
OBJECT_YAML_PATH = os.path.join(PACKAGE_DIR, 'config', 'object.yaml')
MODEL_PT_PATH = os.path.join(PACKAGE_DIR, 'mask-rcnn.pt')


class ObjectEntity:
    def __init__(self, name: str, id: int, colour: list):
        self.name = name
        self.id = id
        self.colour = colour

    def __str__(self):
        return "Name: {}, ID: {}, Colour: {}".format(self.name, self.id, self.colour)

class LabelManager:
    def __init__(self):
        print("ObjectPath: ", OBJECT_YAML_PATH)
        with open(OBJECT_YAML_PATH, 'r') as file:
            config = yaml.safe_load(file)
        
        self.obj_entity_map = {}
        self.class_names = []
        for ele in config['DRIVING_objects']:
            name = ele['Entity']
            id = ele['ID']
            colour = ele['Colour']
            entity = ObjectEntity(name, id=id, colour=colour)
            self.obj_entity_map.update({id: entity})
            self.class_names.append(name)
    
    def get_colour(self, id:int):
        return self.obj_entity_map[id].colour
    

    def get_num(self):
        return len(list(self.obj_entity_map.keys()))
    
    def get_ids(self):
        return list(self.obj_entity_map.keys())
    
    def get_names(self):
        return self.class_names
    
    def get_name(self, id:int):
        return self.class_names[id]
    
    def get_label(self, name:str):
        index = self.class_names.index(name)
        return index + 1
    
class ImageSegment:
    def __init__(self):
        self.label_manager = LabelManager()
        self.num_classes = self.label_manager.get_num() + 1
        self.model = self.build_default_model(self.num_classes)
        # self.model.load_state_dict(torch.load('rev_model.pt', weights_only=True))
        self.model.load_state_dict(torch.load(MODEL_PT_PATH, weights_only=True))
        self.model.eval()

        self.class_names = ['__background__'] + self.label_manager.get_names()
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        self.model.to(self.device)
        # Transform
        self.transform = T.Compose([T.ToTensor()])
        self.entities = [1, 2, 3, 16, 17]

    
    def build_default_model(self, num_classes):
        print('build_model - num_classes: ', num_classes)
        # load an instance segmentation model pre-trained on COCO
        model = torchvision.models.detection.maskrcnn_resnet50_fpn_v2(pre_trained = False)

        # get the number of input features for the classifier
        in_features = model.roi_heads.box_predictor.cls_score.in_features
        # replace the pre-trained head with a new one
        model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)

        # Stop here if you are fine-tunning Faster-RCNN

        # now get the number of input features for the mask classifier
        in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
        hidden_layer = 256
        # and replace the mask predictor with a new one
        model.roi_heads.mask_predictor = MaskRCNNPredictor(in_features_mask,
                                                            hidden_layer,
                                                            num_classes)
        return model

    def get_prediction(self, img, confidence):
        img = self.transform(img)

        img = img.to(self.device)
        pred = self.model([img])
        pred_score = list(pred[0]['scores'].detach().cpu().numpy())
        pred_list = [pred_score.index(x) for x in pred_score if x>confidence]
        if (len(pred_list) == 0):
            return [], [], [], []
        pred_t = pred_list[-1]
        masks = (pred[0]['masks']>0.5).squeeze().detach().cpu().numpy()

        pred_class = [self.class_names[i] for i in list(pred[0]['labels'].cpu().numpy())]
        pred_boxes = [[(i[0], i[1]), (i[2], i[3])] for i in list(pred[0]['boxes'].detach().cpu().numpy())]
        masks = masks[:pred_t+1]
        pred_boxes = pred_boxes[:pred_t+1]
        pred_class = pred_class[:pred_t+1]
        labels = pred[0]['labels'].cpu().numpy()
        labels = labels[:pred_t+1]

        return masks, pred_boxes, pred_class, labels
    
    def segment_instance(self, image, confidence=0.7, rect_th=1, text_size=0.5, text_th=1):
        start_time = time.perf_counter()
        masks, boxes, pred_cls, labels = self.get_prediction(image, confidence)
        end_time = time.perf_counter()
        #print('detecting labels: ', labels, 'processing_time: ', end_time-start_time)
        # img = cv2.imread(img_path)
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h = image.height
        w = image.width
        out_img = np.zeros((h, w, 4), np.uint8)
        img_array = np.array(image)
        
        for i in range(len(masks)):
            # rgb_mask = get_coloured_mask(masks[i])
            # img = cv2.addWeighted(img, 0.8, rgb_mask, 0.3, 0)
            label = labels[i]
            colour = self.label_manager.get_colour(label)
            # Efficiently apply color and alpha to the mask using NumPy
            mask = masks[i] > 0  # Create a boolean mask where the condition is true
            
            colored_mask = np.array(colour + [153], dtype=np.uint8)
            out_img[mask] = colored_mask
            # for y in range(0, h):
            #     for x in range(0, w):
            #         if (masks[i][y, x] > 0):
            #             # alpha 60% = 153
            #             out_img[y, x] = colour + [153]
            if (label in self.entities):
                pt1 = tuple([int(j) for j in boxes[i][0]])
                pt2 = tuple([int(j) for j in boxes[i][1]])
                #print('pt1: ', pt1, 'pt2: ', pt2)
                cv2.rectangle(img_array, pt1, pt2,color=colour, thickness=rect_th)
                cv2.putText(img_array,pred_cls[i], pt1, cv2.FONT_HERSHEY_SIMPLEX, text_size, colour,thickness=text_th)

        return Image.fromarray(img_array), Image.fromarray(out_img, mode='RGBA'), pred_cls, labels

class SegmentationPublisher(Node):

    def __init__(self):
        super().__init__('SegmentationPublisher')
        # Subscriber
        self.img_subscription = self.create_subscription(
            RosImage,
            "/CameraFront",
            self.img_listener_callback,
            10)
        # Publisher
        self.publisher_ = self.create_publisher(RosImage, 'segment_image', 10)

        # Timer
        self.timer = self.create_timer(0.066, self.timer_cb)

        # Variables
        self.img_segment = ImageSegment()
        self.cur_img = None
        self.cur_stamp = None
        self.pre_stamp = None
        
    def img_listener_callback(self, msg):
        #print("Got something!")
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_img_h, cv_img_w = cv_img.shape
        #print(cv_img.shape)
        #self.get_logger().info(f"Image Shape: {cv_img.shape}")
        resize_h=int(cv_img_h*0.2)
        img=cv2.resize(cv_img[resize_h:,:],(480,240))
        self.cur_img = img
        self.cur_stamp = msg.header.stamp
        #self.get_logger().info("CV Image resize_h %d" % (resize_h))

    def timer_cb(self):
        if self.cur_img is None:
            return
        if self.pre_stamp is not None and self.pre_stamp == self.cur_stamp:
            return 
        image = Image.fromarray(self.cur_img)
        _, mask, _, _ = self.img_segment.segment_instance(image)
        mask_array = np.array(mask)
        cv_img = cv2.cvtColor(mask_array, cv2.COLOR_RGBA2BGRA)
        self.publish_msg(cv_img)
        self.pre_stamp = self.cur_stamp
    
    def publish_msg(self, img):
        ros_image_message = bridge.cv2_to_imgmsg(img, "bgra8")
        ros_image_message.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(ros_image_message)

def main():
    rclpy.init()
    rclpy.spin(SegmentationPublisher())
    rclpy.shutdown()
