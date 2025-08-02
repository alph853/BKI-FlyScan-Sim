import csv
import os
import glob
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Dict, Set, Optional
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R
import numpy as np


TYPE_TO_URI_MAPPING: Dict[str, str] = {
    'shelf': "https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf",
    'shelf_big': "https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf_big",
}


def pose_to_matrix(x, y, z, roll, pitch, yaw):
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T

def matrix_to_pose(T):
    x, y, z = T[:3, 3]
    roll, pitch, yaw = R.from_matrix(T[:3, :3]).as_euler('xyz')
    return [x, y, z, roll, pitch, yaw]

def transform_pose_to_parent(child_pose_world, parent_pose_world):
    T_child_world = pose_to_matrix(*child_pose_world)
    T_parent_world = pose_to_matrix(*parent_pose_world)
    T_parent_world_inv = np.linalg.inv(T_parent_world)

    T_child_in_parent = T_parent_world_inv @ T_child_world
    return matrix_to_pose(T_child_in_parent)


def get_model_uri(model_id: str) -> Optional[str]:
    """Get appropriate URI for model type"""
    # Handle compound model types like 'shelf_big'
    if model_id.startswith('shelf_big'):
        base_type = 'shelf_big'
    else:
        base_type = model_id.split('_')[0]
    return TYPE_TO_URI_MAPPING.get(base_type)


def get_qr_images(models_dir: str) -> List[str]:
    """Get list of available QR images"""
    qr_dir = os.path.join(models_dir, "QR")
    if not os.path.exists(qr_dir):
        # Fallback to default if QR directory doesn't exist
        return ["models/QR/qr.png"]
    
    qr_files = glob.glob(os.path.join(qr_dir, "*.png")) + glob.glob(os.path.join(qr_dir, "*.jpg"))
    if not qr_files:
        return ["models/QR/qr.png"]
    
    # Convert to relative paths
    return [f"models/QR/{os.path.basename(f)}" for f in qr_files]


def include_model(name: str, pose: List[float], uri: str) -> ET.Element:
    """Create model XML element"""
    include = ET.Element("include")

    uri_elem = ET.SubElement(include, "uri")
    uri_elem.text = uri
    
    name_elem = ET.SubElement(include, "name")
    name_elem.text = name

    pose_elem = ET.SubElement(include, "pose")
    pose_elem.text = " ".join(map(str, pose))

    return include


def create_composite_model(parent_entry: 'ModelEntry', children: List['ModelEntry'], parent_uri: str, qr_images: List[str]) -> ET.Element:
    """Create a composite model with parent and child models using external URI for parent"""
    # Check if all children are QR codes (leaf nodes)
    model = ET.Element("model", name=parent_entry.id)

    static_elem = ET.SubElement(model, "static")
    static_elem.text = "true"
    
    pose_elem = ET.SubElement(model, "pose")
    pose_elem.text = " ".join(map(str, parent_entry.pose))

    # Include the parent model using its URI
    parent_include = ET.SubElement(model, "include")
    parent_uri_elem = ET.SubElement(parent_include, "uri")
    parent_uri_elem.text = parent_uri
    parent_name_elem = ET.SubElement(parent_include, "name")
    parent_name_elem.text = "shelf_model"
    parent_pose_elem = ET.SubElement(parent_include, "pose")
    parent_pose_elem.text = "0 0 0 0 0 0"  # Relative to model frame
    
    # Add QR codes as simple visual elements in links with world frame poses
    for i, child_model in enumerate(children):
        if child_model.get_base_type() != 'qr':
            raise NotImplementedError(f"Composite models with non-QR children not implemented. Found child type: {child_model.get_base_type()}")

        # Create QR link (required for SDF compliance)
        qr_link = ET.SubElement(model, "link", name=f"qr_link_{i}")
        
        # Set link pose in world frame (no transformation)
        qr_link_pose = ET.SubElement(qr_link, "pose")

        relative_pose = transform_pose_to_parent(child_model.pose, parent_entry.pose)
        qr_link_pose.text = " ".join(map(str, relative_pose))
        
        # Create visual element within the link
        qr_visual = ET.SubElement(qr_link, "visual", name="qr_visual")
        
        qr_geometry = ET.SubElement(qr_visual, "geometry")
        qr_plane = ET.SubElement(qr_geometry, "plane")
        qr_normal = ET.SubElement(qr_plane, "normal")
        qr_normal.text = "0 0 1"    # Normal pointing up
        qr_size = ET.SubElement(qr_plane, "size")
        qr_size.text = "0.2 0.2"
        
        qr_material = ET.SubElement(qr_visual, "material")
        qr_ambient = ET.SubElement(qr_material, "ambient")
        qr_ambient.text = "1 1 1 1"
        qr_diffuse = ET.SubElement(qr_material, "diffuse")
        qr_diffuse.text = "1 1 1 1"
        
        # PBR material with cycling QR images
        pbr = ET.SubElement(qr_material, "pbr")
        metal = ET.SubElement(pbr, "metal")
        albedo_map = ET.SubElement(metal, "albedo_map")
        # Cycle through available QR images
        qr_image = qr_images[i % len(qr_images)]
        albedo_map.text = qr_image
    
    return model


@dataclass
class ModelEntry:
    id: str
    pose: List[float]
    parent_frame: str
    children: List['ModelEntry'] = field(default_factory=list)
    
    def get_base_type(self) -> str:
        """Extract base type from model ID"""
        # Handle compound model types like 'shelf_big'
        if self.id.startswith('shelf_big'):
            return 'shelf_big'
        else:
            return self.id.split('_')[0]
    
def generate_xml_models(csv_file_path: str, world_file_path: str) -> None:
    """Inject SDF models from CSV data directly into warehouse world file"""
    
    # Read and parse CSV data
    rows: List[Dict[str, str]] = []
    with open(csv_file_path, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        all_rows = list(reader)
        rows = [r for r in all_rows if 
                r.get('id', '').strip() and 
                r.get('pose_in_world_frame', '').strip() and 
                r.get('target_parent_frame', '').strip()]
        if len(rows) != len(all_rows):
            raise ValueError("CSV file contains empty or invalid rows.")

    all_models: List[ModelEntry] = []
    
    for row in rows:
        model_id = row['id'].strip()
        parent_frame = row['target_parent_frame'].strip()
        pose = [float(x) for x in row['pose_in_world_frame'].strip().split()]
        all_models.append(ModelEntry(id=model_id, pose=pose, parent_frame=parent_frame))

    # Build hierarchical tree structure
    models_by_id: Dict[str, ModelEntry] = {model.id: model for model in all_models}
    
    # Build parent-child relationships
    for model in all_models:
        if model.parent_frame != 'world' and model.parent_frame in models_by_id:
            models_by_id[model.parent_frame].children.append(model)
    
    # Validate that all non-world parents exist
    for model in all_models:
        if model.parent_frame != 'world' and model.parent_frame not in models_by_id:
            raise ValueError(f"Parent frame '{model.parent_frame}' not found for model '{model.id}'.")
    
    # Get root models (those with world as parent)
    root_models = [model for model in all_models if model.parent_frame == 'world']

    # Get QR images
    sim_pkg = get_package_share_directory('flyscan_simulation')
    models_dir = os.path.join(sim_pkg, 'worlds', 'models')
    qr_images = get_qr_images(models_dir)
    
    # Read existing world file
    world_tree = ET.parse(world_file_path)
    world_root = world_tree.getroot()
    world_element = world_root.find('world')
    
    # Find and remove existing models with same names (overwrite instead of duplicate)
    model_names_to_add = set()
    for model in all_models:
        if model.parent_frame == 'world':
            model_names_to_add.add(model.id)
    
    # Remove existing models and includes with same names
    elements_to_remove = []
    for element in world_element:
        if element.tag in ['model', 'include']:
            name_elem = element.get('name')
            if name_elem is None:
                name_child = element.find('name')
                if name_child is not None:
                    name_elem = name_child.text
            if name_elem and name_elem in model_names_to_add:
                elements_to_remove.append(element)
    
    for element in elements_to_remove:
        world_element.remove(element)

    def is_leaf_node(model: ModelEntry) -> bool:
        """Check if a model is a leaf node (has no children)"""
        return len(model.children) == 0
    
    def has_only_leaf_children(model: ModelEntry) -> bool:
        """Check if all children are leaf nodes"""
        return all(is_leaf_node(child) for child in model.children)
    
    def process_model_hierarchy(model: ModelEntry) -> None:
        """Recursively process model hierarchy"""
        uri = get_model_uri(model.id)
        if uri is None:
            # Process children even if parent has no URI (for organizational nodes)
            for child in model.children:
                process_model_hierarchy(child)
            return
        
        # Check if model has children
        if model.children:
            if has_only_leaf_children(model):
                # Check if all leaf children are QR codes
                composite_model = create_composite_model(model, model.children, uri, qr_images)
                world_element.append(composite_model)
            else:
                # Has non-leaf children - process separately
                model_include = include_model(model.id, model.pose, uri)
                world_element.append(model_include)
                
                # Process non-leaf children recursively
                for child in model.children:
                    if not is_leaf_node(child):
                        process_model_hierarchy(child)
        else:
            # Leaf model without children
            if model.get_base_type() != 'qr':  # QR codes are handled by their parents
                model_include = include_model(model.id, model.pose, uri)
                world_element.append(model_include)
    
    # Process all root models
    for root_model in root_models:
        process_model_hierarchy(root_model)
    
    # Write modified world file
    ET.indent(world_tree, space="  ", level=0)
    world_tree.write(world_file_path, encoding='utf-8', xml_declaration=True)
    
    print(f"Injected {len(root_models)} root models into {world_file_path}")



def main():
    """Main function to generate qr_stocks.sdf"""
    sim_pkg = get_package_share_directory('flyscan_simulation')
    worlds_path  = os.path.join(sim_pkg, 'worlds')

    csv_file = os.path.join(worlds_path, "models", "model_poses_in_world_frame.csv")
    world_file = os.path.join(worlds_path, "warehouse_outdoor.sdf")
    
    generate_xml_models(csv_file, world_file)


if __name__ == "__main__":
    main()