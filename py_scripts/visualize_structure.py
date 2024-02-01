from utils import *

def visualize_structure(config_fname="./config.json"):
    config = load_json(config_fname)
    brick_library = load_json(config["Brick_Library_fname"])
    world_dim = config["World_Dimension"]
    lego_structure = load_json(config["Lego_fname"])
    for key in lego_structure.keys():
        lego_structure[key]["z"] += 1
    world_grid = construct_world_grid(lego_structure, world_dim, brick_library) 
    ax = plt.figure().add_subplot(projection='3d')
    ax.voxels(world_grid)
    plt.show()

if __name__ == '__main__':
    visualize_structure()