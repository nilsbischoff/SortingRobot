import numpy as np
import libry as ry

__all__ = [
    'get_random_color',
    'get_random_position',
    'get_random_quaternion',
    'get_reachable_baskets',
    'get_color_from_color_code',
    'get_random_shape'
]


def get_color_from_color_code(color_code):
    """Converts a color code to a color name.

    Args:
        color_code (list): color code

    Returns:
        str: color name
    """
    if color_code[0]:
        return 'red'
    elif color_code[1]:
        return 'green'
    elif color_code[2]:
        return 'blue'
    elif not all(color_code):
        return 'black'
    else:
        return 'unknown'


def get_prior_knowledge():
    """Returns some prior knowledge about the scene.

    Returns:
        list: color codes which are allowed for the objects
        dict: coordinate intervals where the objects are allowed to spawn
        tuple: colors of reachable bins for each robot arm
    """
    allowed_colors = [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ]

    working_space = {
        'x': [-1, 1],
        'y': [0.4, 0.5]
    }

    l_reachable_baskets = ['red', 'blue']

    r_reachable_baskets = ['red', 'green']

    possible_shapes = {
        ry.ST.sphere: [[0.03, 0.05]],
        ry.ST.capsule: [[0.2, 0.5], [0.02, 0.04]]
    }

    return allowed_colors, working_space, (l_reachable_baskets, r_reachable_baskets), possible_shapes


def get_random_color():
    """Returns a random color code of the allowed ones.

    Returns:
        list: color code
    """
    colors, _, _, _ = get_prior_knowledge()
    idx = np.random.choice(range(len(colors)))

    return colors[idx]


def get_random_position():
    """Returns a random position for spawning in the allowed workspace.

    Returns:
        list: random position
    """
    _, ws, _, _ = get_prior_knowledge()
    x = np.random.uniform(ws['x'][0], ws['x'][1])
    y = np.random.uniform(ws['y'][0], ws['y'][1])

    return [x, y, 1]


def get_random_quaternion():
    """Returns a random unit quaternion as orientation.

    Returns:
        list: unit quaternion
    """
    quat = np.random.uniform(-1.0, 1.0, size=(4,))
    quat /= np.linalg.norm(quat)

    return quat.tolist()


def get_random_shape():
    """Returns a random shape of the allowed ones.

    Returns:
        libry.ST: shape
        list: parameters of the shape
    """
    _, _, _, possible_shapes = get_prior_knowledge()

    idx = np.random.choice(range(len(possible_shapes)))

    shape = list(possible_shapes)[idx]

    params = []
    for param in possible_shapes[shape]:
        params.append(np.random.uniform(param[0], param[1]))

    return shape, params


def get_reachable_baskets(arm):
    """Returns reachable baskets for the specified arm.

    Args:
        arm (str): prefix of robot arm

    Returns:
        list: colors of reachable baskets
    """
    _, _, (l_baskets, r_baskets), _ = get_prior_knowledge()

    if arm == 'L':
        return l_baskets
    elif arm == 'R':
        return r_baskets
    else:
        raise NotImplementedError
