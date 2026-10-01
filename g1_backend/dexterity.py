"""Optional end-effector selection for generated simulation descriptions."""

import copy
import xml.etree.ElementTree as ET

from .dex1 import ASSETS, verify_assets


def description_with_claws(xml):
    """Replace rubber hands with the same official Dex1 links used in MuJoCo."""
    verify_assets()
    root = ET.fromstring(xml)
    source = ET.parse(ASSETS / 'g1_29dof_mode_15_with_dex1_1.urdf').getroot()
    for side in ('left', 'right'):
        for element in list(root):
            child = element.find('child')
            if (element.tag == 'link' and element.get('name') == f'{side}_rubber_hand') or (
                element.tag == 'joint' and child is not None
                and child.get('link') == f'{side}_rubber_hand'
            ):
                root.remove(element)
        for element in source:
            if element.tag not in ('link', 'joint'):
                continue
            child = element.find('child')
            selected = (element.get('name', '') if element.tag == 'link'
                        else child.get('link', '') if child is not None else '')
            if not selected.startswith(f'{side}_dex1_'):
                continue
            element = copy.deepcopy(element)
            for mesh in element.findall('.//mesh'):
                mesh.set('filename', (ASSETS / mesh.get('filename').split('/')[-1]).as_uri())
            root.append(element)
    return ET.tostring(root, encoding='unicode')
