# conf.py
import sys
import os
from pathlib import Path

# Configuración base
project = 'Gymbrot'
copyright = '2025, Gymbrot Team'
author = 'Gymbrot Team'

# Afegeix aquestes importacions a la part superior
from sphinx.ext.intersphinx import inspect_main
import sys
from pathlib import Path

# Configuració Intersphinx actualitzada
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'rclpy': ('https://docs.ros2.org/latest/api/rclpy/', None),
    'geometry_msgs': ('https://docs.ros2.org/latest/api/geometry_msgs', None),
    'nav2_msgs': ('https://docs.ros2.org/latest/api/nav2_msgs', None),
    'action_msgs': ('https://docs.ros2.org/latest/api/action_msgs', None),
    'std_msgs': ('https://docs.ros2.org/latest/api/std_msgs', None)
}

# Configuració AutoAPI específica per ROS2
autoapi_options = [
    'members',
    'undoc-members',
    'show-inheritance',
    'show-module-summary',
    'special-members',
    'imported-members',
    'implicit-namespaces'
]

# Afegeix aquesta línia per a tipus de ROS2
nitpick_ignore = [
    ('py:class', 'action_msgs.msg.GoalStatus'),
    ('py:class', 'optional'),
    ('py:class', 'TypeVar'),
    ('py:class', 'Any'),
    ('py:class', 'nav2_msgs.action.NavigateToPose_FeedbackMessage')
]

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.napoleon',
    'sphinx.ext.intersphinx',
    'autoapi.extension',
    'sphinxcontrib.mermaid',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
language = 'es'

# Configuración HTML
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_css_files = ['custom.css']
html_theme_options = {
    'logo_only': True,
    'display_version': False,
}

# Configuración de rutas
base_dir = Path(__file__).resolve().parent.parent.parent  # Asumiendo doc/conf.py -> turtlebot3_ws/src/

sys.path.insert(0, str(base_dir / 'gymbrot'))
sys.path.insert(0, str(base_dir / 'gymbrot_interfaces'))
sys.path.insert(0, str(base_dir / 'gymbrot_point_nav_system'))

# Configuración AutoAPI
autoapi_type = 'python'
autoapi_dirs = [
    str(base_dir / 'gymbrot'),
    str(base_dir / 'gymbrot_interfaces'),
    str(base_dir / 'gymbrot_point_nav_system')
]
autoapi_ignore = ['*test*', '*__pycache__*', '*node_modules*']

# Intersphinx mapping
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
}