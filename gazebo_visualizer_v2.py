"""
gazebo_visualizer_v2.py — Runway World Uyumlu NPC Görselleştirici

Değişiklikler:
- World adı: "runway" (vtail_runway.sdf ile uyumlu)
- NPC modelleri: basitleştirilmiş sabit kanat uçak (mini_talon benzeri)
- Koordinat sistemi: spherical_coordinates ile Gazebo ENU dönüşümü
- Yaw rotasyonu: NPC heading'ine göre model döndürme
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import time
import os

try:
    import gz.transport13
    from gz.msgs10 import (
        pose_pb2 as gz_pose_pb2,
        boolean_pb2 as gz_bool_pb2,
        entity_factory_pb2 as gz_entity_factory_pb2,
        entity_pb2 as gz_entity_pb2
    )
    GZ_TRANSPORT_AVAILABLE = True
except ImportError:
    GZ_TRANSPORT_AVAILABLE = False


# ─── Sabitler ───
WORLD_NAME = "runway"
SERVICE_CREATE = f"/world/{WORLD_NAME}/create"
SERVICE_REMOVE = f"/world/{WORLD_NAME}/remove"
SERVICE_SET_POSE = f"/world/{WORLD_NAME}/set_pose"


def heading_to_quaternion(heading_deg):
    """Heading (derece, 0=kuzey CW) → Gazebo quaternion (ENU, 0=doğu CCW)"""
    # Heading'i ENU yaw'a çevir: yaw_enu = 90 - heading
    yaw = math.radians(90.0 - heading_deg)
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw


def npc_plane_sdf(name, color_r=0.8, color_g=0.2, color_b=0.2):
    """Basitleştirilmiş sabit kanat uçak SDF (mini_talon benzeri boyutlar)"""
    col = f"{color_r} {color_g} {color_b} 1"
    dark = f"{color_r*0.5} {color_g*0.5} {color_b*0.5} 1"
    return f"""<sdf version='1.6'>
  <model name='{name}'>
    <static>false</static>
    <link name='body'>
      <gravity>false</gravity>
      <kinematic>true</kinematic>

      <!-- Gövde -->
      <visual name='fuselage'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box><size>0.7 0.12 0.10</size></box>
        </geometry>
        <material>
          <ambient>{col}</ambient><diffuse>{col}</diffuse>
          <emissive>{dark}</emissive>
        </material>
      </visual>

      <!-- Sol kanat -->
      <visual name='left_wing'>
        <pose>0 0.35 0.02 0 0 0</pose>
        <geometry>
          <box><size>0.25 0.60 0.02</size></box>
        </geometry>
        <material>
          <ambient>{col}</ambient><diffuse>{col}</diffuse>
          <emissive>{dark}</emissive>
        </material>
      </visual>

      <!-- Sağ kanat -->
      <visual name='right_wing'>
        <pose>0 -0.35 0.02 0 0 0</pose>
        <geometry>
          <box><size>0.25 0.60 0.02</size></box>
        </geometry>
        <material>
          <ambient>{col}</ambient><diffuse>{col}</diffuse>
          <emissive>{dark}</emissive>
        </material>
      </visual>

      <!-- Sol V-kuyruk -->
      <visual name='left_vtail'>
        <pose>-0.30 0.08 0.06 0.3 0 0</pose>
        <geometry>
          <box><size>0.12 0.15 0.015</size></box>
        </geometry>
        <material>
          <ambient>{col}</ambient><diffuse>{col}</diffuse>
        </material>
      </visual>

      <!-- Sağ V-kuyruk -->
      <visual name='right_vtail'>
        <pose>-0.30 -0.08 0.06 -0.3 0 0</pose>
        <geometry>
          <box><size>0.12 0.15 0.015</size></box>
        </geometry>
        <material>
          <ambient>{col}</ambient><diffuse>{col}</diffuse>
        </material>
      </visual>

      <collision name='col'>
        <geometry><box><size>0.7 1.2 0.15</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>"""


# Her takım için farklı renk
TEAM_COLORS = [
    (0.9, 0.1, 0.1),   # 1 kırmızı
    (0.1, 0.1, 0.9),   # 2 mavi
    (0.1, 0.8, 0.1),   # 3 yeşil
    (0.9, 0.6, 0.0),   # 4 turuncu
    (0.7, 0.0, 0.7),   # 5 mor
    (0.0, 0.8, 0.8),   # 6 cyan
    (0.9, 0.9, 0.0),   # 7 sarı
    (0.5, 0.3, 0.1),   # 8 kahverengi
    (1.0, 0.4, 0.7),   # 9 pembe
    (0.3, 0.3, 0.3),   # 10 gri
    (0.6, 0.9, 0.3),   # 11 açık yeşil
    (0.3, 0.3, 0.9),   # 12 koyu mavi
    (0.9, 0.3, 0.6),   # 13 magenta
    (0.1, 0.5, 0.5),   # 14 teal
    (0.8, 0.8, 0.8),   # 15 beyaz
]


class GazeboVisualizerV2(Node):
    def __init__(self):
        super().__init__('gazebo_visualizer')
        self.subscription = self.create_subscription(
            String, '/sunucu_telemetri', self.listener_callback, 10)

        # Referans koordinatları (vtail_runway.sdf spherical_coordinates)
        self.ref_lat = -35.363262
        self.ref_lon = 149.165237
        self.spawned_uavs = set()
        self.arena_setup = False

        if GZ_TRANSPORT_AVAILABLE:
            self.gz_node = gz.transport13.Node()
        else:
            self.gz_node = None
            self.get_logger().warn('gz.transport13 bulunamadı!')

        self.cleanup_old_entities()
        self.get_logger().info(f'Visualizer başlatıldı. World: {WORLD_NAME}')
        self.get_logger().info(f'Referans: lat={self.ref_lat}, lon={self.ref_lon}')

        # boundary.json'dan referans güncelle
        self.load_boundary_ref()

    def load_boundary_ref(self):
        """boundary.json varsa, NPC'lerin merkezini referans al"""
        for path in ['boundary.json', '/ros2_ws/src/siha_telemetri/boundary.json']:
            if os.path.exists(path):
                try:
                    with open(path) as f:
                        data = json.load(f)
                        if 'boundary' in data and len(data['boundary']) >= 3:
                            lats = [p[0] for p in data['boundary']]
                            lons = [p[1] for p in data['boundary']]
                            self.ref_lat = min(lats)
                            self.ref_lon = min(lons)
                            self.get_logger().info(
                                f'Referans güncellendi: lat={self.ref_lat}, lon={self.ref_lon}')
                            if not self.arena_setup:
                                self.setup_arena(data['boundary'])
                                self.arena_setup = True
                except Exception as e:
                    self.get_logger().error(f'Boundary okuma hatası: {e}')

    def cleanup_old_entities(self):
        if not (self.gz_node and GZ_TRANSPORT_AVAILABLE):
            return
        entities = [f"npc_{i}" for i in range(1, 16)]
        entities += [f"arena_wall_{i}" for i in range(20)]
        entities.append("arena_center")
        for name in entities:
            req = gz_entity_pb2.Entity()
            req.name = name
            req.type = gz_entity_pb2.Entity.MODEL
            self.gz_node.request(
                SERVICE_REMOVE, req,
                gz_entity_pb2.Entity, gz_bool_pb2.Boolean, 100)
        time.sleep(0.5)

    def setup_arena(self, boundary_coords):
        """Yarışma sınırı duvarlarını çiz"""
        if not boundary_coords or len(boundary_coords) < 3:
            return
        n = len(boundary_coords)
        for i in range(n):
            p1 = boundary_coords[i]
            p2 = boundary_coords[(i + 1) % n]
            x1 = (p1[1] - self.ref_lon) * 111320.0 * math.cos(math.radians(self.ref_lat))
            y1 = (p1[0] - self.ref_lat) * 111320.0
            x2 = (p2[1] - self.ref_lon) * 111320.0 * math.cos(math.radians(self.ref_lat))
            y2 = (p2[0] - self.ref_lat) * 111320.0
            length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            mid_x = (x1 + x2) / 2.0
            mid_y = (y1 + y2) / 2.0
            yaw = math.atan2(y2 - y1, x2 - x1)
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            wall_name = f"arena_wall_{i}"
            wall_sdf = (f"<sdf version='1.6'><model name='{wall_name}'><static>true</static>"
                        f"<link name='l'><visual name='v'><geometry><box>"
                        f"<size>{length} 3.0 100.0</size></box></geometry>"
                        f"<material><ambient>1 0 0 0.4</ambient><diffuse>1 0 0 0.4</diffuse>"
                        f"<emissive>1 0 0 0.3</emissive></material></visual></link></model></sdf>")
            self.spawn_entity(wall_name, wall_sdf, mid_x, mid_y, 50, 0, 0, qz, qw)

    def gps_to_local(self, lat, lon, alt):
        """GPS → Gazebo local ENU koordinatları"""
        x = (lon - self.ref_lon) * 111320.0 * math.cos(math.radians(self.ref_lat))
        y = (lat - self.ref_lat) * 111320.0
        z = alt
        return x, y, z

    def spawn_entity(self, name, sdf, px, py, pz, ox, oy, oz, ow):
        if not (self.gz_node and GZ_TRANSPORT_AVAILABLE):
            return
        try:
            req = gz_entity_factory_pb2.EntityFactory()
            req.sdf = sdf
            req.pose.position.x = float(px)
            req.pose.position.y = float(py)
            req.pose.position.z = float(pz)
            req.pose.orientation.x = float(ox)
            req.pose.orientation.y = float(oy)
            req.pose.orientation.z = float(oz)
            req.pose.orientation.w = float(ow)
            self.gz_node.request(
                SERVICE_CREATE, req,
                gz_entity_factory_pb2.EntityFactory, gz_bool_pb2.Boolean, 500)
        except Exception as e:
            self.get_logger().error(f"Spawn hatası ({name}): {e}")

    def move_entity(self, name, x, y, z, heading_deg=0):
        if not (self.gz_node and GZ_TRANSPORT_AVAILABLE):
            return
        try:
            ox, oy, oz, ow = heading_to_quaternion(heading_deg)
            req = gz_pose_pb2.Pose()
            req.name = name
            req.position.x = float(x)
            req.position.y = float(y)
            req.position.z = float(z)
            req.orientation.x = float(ox)
            req.orientation.y = float(oy)
            req.orientation.z = float(oz)
            req.orientation.w = float(ow)
            self.gz_node.request(
                SERVICE_SET_POSE, req,
                gz_pose_pb2.Pose, gz_bool_pb2.Boolean, 250)
        except Exception:
            pass

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            uav_list = data.get("konumBilgileri", data.get("konum_bilgileri", []))

            # İlk veri geldiğinde boundary kontrolü yap
            if not self.arena_setup:
                self.load_boundary_ref()

            for npc in uav_list:
                tid = npc.get("takim_numarasi", npc.get("takimNumarasi"))
                lat = npc.get("iha_enlem", npc.get("IHA_enlem", 0.0))
                lon = npc.get("iha_boylam", npc.get("IHA_boylam", 0.0))
                alt = npc.get("iha_irtifa", npc.get("IHA_irtifa", 0.0))
                hdg = npc.get("iha_yonelme", npc.get("IHA_yonelme", 0))

                x, y, z = self.gps_to_local(lat, lon, alt)
                name = f"npc_{tid}"

                if tid not in self.spawned_uavs:
                    # İlk kez → model oluştur
                    color_idx = (tid - 1) % len(TEAM_COLORS)
                    r, g, b = TEAM_COLORS[color_idx]
                    sdf = npc_plane_sdf(name, r, g, b)
                    ox, oy, oz, ow = heading_to_quaternion(hdg)
                    self.spawn_entity(name, sdf, x, y, z, ox, oy, oz, ow)
                    self.spawned_uavs.add(tid)
                    self.get_logger().info(f'NPC #{tid} spawn edildi (x={x:.0f}, y={y:.0f}, z={z:.0f})')
                else:
                    # Güncelle
                    self.move_entity(name, x, y, z, hdg)

        except Exception as e:
            self.get_logger().error(f'Callback hatası: {e}')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GazeboVisualizerV2())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
