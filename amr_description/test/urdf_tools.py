"""Utilidades de prueba: procesar el xacro y calcular cinemática directa desde la URDF."""
import math
import os
import pathlib
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

PKG_DIR = pathlib.Path(__file__).resolve().parents[1]
XACRO = PKG_DIR / 'urdf' / 'amr.urdf.xacro'


def _ensure_package_findable():
    """$(find amr_description) necesita el índice de ament; si el paquete aún no
    está instalado (p. ej. pytest directo sobre el código fuente) se crea un
    prefijo temporal que apunta a este directorio."""
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('amr_description')
        return
    except Exception:  # noqa: BLE001 - PackageNotFoundError o ament no disponible
        pass
    prefix = pathlib.Path(tempfile.mkdtemp(prefix='amr_desc_'))
    marker = prefix / 'share' / 'ament_index' / 'resource_index' / 'packages'
    marker.mkdir(parents=True)
    (marker / 'amr_description').write_text('')
    os.symlink(PKG_DIR, prefix / 'share' / 'amr_description')
    os.environ['AMENT_PREFIX_PATH'] = (
        f"{prefix}{os.pathsep}{os.environ.get('AMENT_PREFIX_PATH', '')}")


def process(**mappings):
    import xacro
    _ensure_package_findable()
    doc = xacro.process_file(str(XACRO), mappings={k: str(v) for k, v in mappings.items()})
    return doc.toxml()


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def homogeneous(rot, xyz):
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = xyz
    return t


def axis_angle(axis, angle):
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * kx @ kx


class UrdfChain:
    def __init__(self, urdf_xml):
        self.root = ET.fromstring(urdf_xml)
        self.joints = {}
        for j in self.root.findall('joint'):
            child = j.find('child').get('link')
            origin = j.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None
                                      else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None
                                      else '0 0 0').split()]
            axis_el = j.find('axis')
            axis = [float(v) for v in axis_el.get('xyz').split()] if axis_el is not None else [1, 0, 0]
            self.joints[child] = {
                'name': j.get('name'), 'type': j.get('type'),
                'parent': j.find('parent').get('link'),
                'xyz': xyz, 'rpy': rpy, 'axis': axis,
            }

    @property
    def link_names(self):
        return {link.get('name') for link in self.root.findall('link')}

    def transform(self, base, tip, q=None):
        """Transformación base->tip con las posiciones articulares q (dict nombre->valor)."""
        q = q or {}
        chain = []
        link = tip
        while link != base:
            if link not in self.joints:
                raise KeyError(f'{tip} no desciende de {base}')
            chain.append(self.joints[link])
            link = self.joints[link]['parent']
        t = np.eye(4)
        for j in reversed(chain):
            t = t @ homogeneous(rot_rpy(*j['rpy']), j['xyz'])
            value = q.get(j['name'], 0.0)
            if j['type'] in ('revolute', 'continuous'):
                t = t @ homogeneous(axis_angle(j['axis'], value), [0, 0, 0])
            elif j['type'] == 'prismatic':
                t = t @ homogeneous(np.eye(3), value * np.asarray(j['axis'], dtype=float))
        return t
