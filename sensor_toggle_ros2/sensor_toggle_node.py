#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from carla_msgs.msg import CarlaEgoVehicleStatus
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from warning_mode_interfaces.action import WarningMode
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import carla
import time
import math
import threading
import asyncio
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

LIDAR_TIMEOUT = 0.5    # 무신호 감지 임계 (초)
CHECK_PERIOD  = 0.1    # 타임아웃 검사 주기 (초)
PUBLISH_RATE  = 10.0   # 제어용 Python API 호출 주기 (Hz)

# 액션 라이브러리 사용해서 behavior Tree로 부터 액션 goal을 받으면 (0, 저속운전 , 1. 갓길 이동 , 2. 차선 평행 회전 , 3. 핸드파킹)

### 위험도 파라미터 #######
K = 3.0 #P에 대한 가중치 ##
lamb = 0.7   # λ      ##
TH = 30.0              ## 100 -> 20으로 수정
########################

def force_all_traffic_lights_green(client):
    world = client.get_world()
    lights = world.get_actors().filter("traffic.traffic_light")

    for light in lights:
        light.set_state(carla.TrafficLightState.Green)
        light.set_green_time(9999.0)
        light.freeze(True)
        print(f"신호등 {light.id} → 초록불 고정")


def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle



class LidarFailSafe(Node):
    def __init__(self):
        super().__init__('lidar_failsafe')
        

        # ① /lidar_alive, /risk_level 퍼블리셔 추가
        self.alive_pub = self.create_publisher(Bool, '/lidar_alive', 10)
        self.risk_pub = self.create_publisher(Float64,'/risk_level',10)
        self.thresh_pub = self.create_publisher(Float64,'/threshold',10)
        # 요청 응답 
        self.pub_w_res = self.create_publisher(Float64,'warningmode_result',  10)
        self.pub_s_res = self.create_publisher(Float64,'shouldershift_result',10)

        # 요청 받기
        self.sub_w_cmd = self.create_subscription(Float64, 'warningmode',   self.cb_warn_cmd,  10)
        self.sub_s_cmd = self.create_subscription(Float64, 'shouldershift', self.cb_shift_cmd, 10)

        self.warn_active   = False
        self.shift_active  = False

        # 차량 센서들 구독
        self.create_subscription( # 라이다
            PointCloud2,
            '/carla/hero/lidar',
            self.lidar_cb,
            10)
        
        self.create_subscription(
            PointCloud2,
            '/carla/hero/semantic_lidar',
            self.semantic_lidar_cb,
            10
        )

        # ③ ROS: 차량 속도(Status) 구독
        self.vehicle_speed = 0.0
        self.create_subscription(
            CarlaEgoVehicleStatus,
            '/carla/hero/vehicle_status',
            self.status_cb,
            10)

        # CARLA Python API 연결
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        force_all_traffic_lights_green(self.client) #강제 초록불
        self.tm = self.client.get_trafficmanager(8000)
        # HERO 차량 찾기 및 Autopilot 비활성
        self.hero = None
        for v in self.world.get_actors().filter('vehicle.*'):
            print(v.id, v.attributes.get('role_name'))
            if v.attributes.get('role_name') == 'hero':
                self.get_logger().info(f"[DEBUG] 차량 ID={v.id}, role_name={v.attributes.get('role_name')}")
                self.hero = v
                #self.hero.set_autopilot(False) #emp
                break
        if not self.hero:
            self.get_logger().error('Hero 차량을 찾을 수 없습니다!')

        # 상태 변수
        self.last_stamp = time.time()
        self.in_fail    = False
        self.current_risk = 0.0
        self.has_parked = False

        # 타이머 설정
        self.create_timer(CHECK_PERIOD, self.check_timeout)
        self.create_timer(1.0 / PUBLISH_RATE, self.publish_ctrl) # 모드 확인해서 실행
        self.create_timer(0.1, self.publish_risk)
        self.create_timer(1.0, self.next_line)
        self.create_timer(0.1, self.publish_th)
        self.create_timer(0.1,self.calculate_risk)
        
        # 초기화 - waypoint와 차선 정보
        self.waypoint = None
        self.right_lane_marking = None
        self.left_lane_marking = None


    def cb_warn_cmd(self, msg: Float64):
        self.warn_active  = msg.data
        if self.warn_active == 1.0:
            self.has_parked = False
            self.get_logger().info(f"[sensor] WarningMode command={msg.data}")
        if self.warn_active == 2.0:
            self.pub_w_res.publish(Float64(data=1.0))
            self.get_logger().info("[sensor] ShoulderShift SUCCESS 전달됨")




    def cb_shift_cmd(self, msg: Float64):
        self.shift_active = msg.data
        if self.shift_active == 1.0:
            self.has_parked = False
            self.get_logger().info(f"[sensor] ShoulderShift command={msg.data}")
            # 여기서 바로 결과 보내기
        if self.shift_active == 2.0:
            self.pub_s_res.publish(Float64(data=1.0))
            self.get_logger().info("[sensor] ShoulderShift SUCCESS 전달됨")

 
    #################################################################################################################

    def define_setP(self):
        # 센서들 영역과 예상 경로 점으로 샘플링해서 유클리드 거리 계산,
        # 연관성 높은 3개 센서를 P집합으로
        pass

    def calculate_risk(self):
        # 모든 센서들 for문돌려서 마지막 타임스탬프 기준 t계산, 고장여부 L을 최종R에 합산하는 함수
        # 우선 hero 차량에 어떤 센서들 있는지 확인
        pass

    def get_lane_lotation(self):
        if self.waypoint:
            self.lane_yaw = self.waypoint.transform.rotation.yaw #차 yaw와 오른쪽 차선 yaw 일치시키기
        

    def next_line(self):
        if self.hero:
            self.waypoint = self.world.get_map().get_waypoint(self.hero.get_location(), project_to_road=True, lane_type=carla.LaneType.Any)
            self.right_lane_marking = self.waypoint.right_lane_marking
            self.left_lane_marking = self.waypoint.left_lane_marking
            self.get_logger().info(f"왼쪽 차선: {self.left_lane_marking.type}, 오른쪽 차선: {self.right_lane_marking.type}")

    def publish_th(self): # 쓰레시홀드 퍼블리시 함수
        threshold = Float64()
        threshold.data = TH
        self.thresh_pub.publish(threshold)

    def publish_risk(self): # 위험도 토픽 퍼블리시 함수
        risk_msg = Float64()
        risk_msg.data = self.current_risk
        self.risk_pub.publish(risk_msg)

    def lidar_cb(self, msg):
        # 라이다 메시지 수신 시점 갱신
        self.last_stamp = time.time() #받았을때 시간을 객체에 저장

        # alive 토픽에 True 발행
        alive_msg = Bool()
        alive_msg.data = True
        self.alive_pub.publish(alive_msg)

        # 만약 이전에 실패 상태였다면 복구 처리
        if self.in_fail:
            self.get_logger().info('Lidar 복구 — 정상 주행으로 전환')
            self.in_fail = False
            self.has_parked = False
            if self.hero:
                self.hero.set_autopilot(True)


    def semantic_lidar_cb(self):
        pass

    def status_cb(self, msg):
        self.vehicle_speed = msg.velocity
        #self.get_logger().info(f'현재 속도: {self.vehicle_speed:.2f} m/s')

    def check_timeout(self):
        t = time.time() - self.last_stamp #(현재 시간 - 최근 수신 시간)
        alive = (t < LIDAR_TIMEOUT)
        if alive: L=0 
        else: L=1
        raw_risk = L * K * math.exp(lamb*t) # 위험도 계산 (라이다만 고려)

        self.current_risk = min(raw_risk, 1000.0)  # 위험도 상한선 1000으로 제한
        self.get_logger().info(f'현재 위험도: {self.current_risk}')
        # ─── /lidar_alive 퍼블리시 ───
        alive_msg = Bool()
        alive_msg.data = alive
        self.alive_pub.publish(alive_msg)

        # ─── 무응답 타임아웃 진입 ───
        if not self.in_fail and self.current_risk > TH:
            self.get_logger().warn(f'위험도 초과 {self.current_risk} — 급정지 모드')
            self.in_fail = True
            if self.hero:
                self.hero.set_autopilot(False)

    def publish_ctrl(self):
        if self.warn_active == 1.0: #저속
            self.tm.vehicle_percentage_speed_difference(self.hero,5.0) #트래픽매니저가 제어
            return

        if self.shift_active == 2.0: 
            self.pub_s_res.publish(Float64(data=1.0))
            return

        if self.shift_active == 1.0: #토픽 값이 1이 되면 페일세이프 기능 on
            if not self.in_fail or not self.hero:
                return
                
            if self.has_parked:
                return

            if not self.waypoint or not self.left_lane_marking or not self.right_lane_marking:
                self.next_line()  # waypoint가 없으면 업데이트
                return
                
            left_type = self.left_lane_marking.type
            right_type = self.right_lane_marking.type

            # 왼쪽 Solid + 오른쪽 None → 평행 맞추고 정지
            if left_type == carla.LaneMarkingType.Solid and right_type == carla.LaneMarkingType.NONE:
                # 차량과 차선의 yaw 차이 계산
                hero_yaw = self.hero.get_transform().rotation.yaw
                lane_yaw = self.waypoint.transform.rotation.yaw
                angle_diff = abs(normalize_angle(hero_yaw - lane_yaw))

                # yaw 차이가 크면 조향 보정
                if angle_diff > 3.0:
                    steer = max(-1.0, min(1.0, normalize_angle(lane_yaw - hero_yaw) / 45.0))
                    ctrl = carla.VehicleControl(throttle=0.2, steer=steer, brake=0.0)
                    self.hero.apply_control(ctrl)
                    self.get_logger().info(f"▶ 평행 맞추는 중 (angle_diff={angle_diff:.2f})")
                    return

                # yaw 일치하면 정지
                ctrl = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, hand_brake=True)
                self.hero.apply_control(ctrl)
                self.has_parked = True
                self.get_logger().info("▶▶▶ 주차 조건 + 방향 일치 → 차량 정지 및 핸드브레이크")
                return

            # 아직 주차 조건 미달 → 우측 이동 계속
            ctrl = carla.VehicleControl(throttle=0.3, steer=0.1, brake=0.0)
            self.hero.apply_control(ctrl)
            self.get_logger().info("▶ 갓길 탐색 중: 우측으로 이동")
        


def main():
    rclpy.init()
    node = LidarFailSafe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()