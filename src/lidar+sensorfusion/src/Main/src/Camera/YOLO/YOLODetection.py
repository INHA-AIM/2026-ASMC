#!/usr/bin/env python3
import cv2
import rospy
import rospkg
import numpy as np
import threading
from sensor_msgs.msg import CompressedImage
from Msg.msg import Yolo_bbox_arr, Yolo_bbox
from ultralytics import YOLO
from pathlib import Path

class st_LaneDetection:
    def __init__(self):
        # --- 가중치 경로: launch 파라미터 우선, 기본값은 Main 패키지 내부 ---
        default_weights = Path(rospkg.RosPack().get_path("Main")) / "YOLOWeight" / "yolo11m.pt"
        weights_path = Path(rospy.get_param("~weights", str(default_weights)))
        if not weights_path.exists():
            rospy.logfatal(f"가중치 파일 없음: {weights_path}")
            raise SystemExit(1)

        # --- 모델 로드 ---
        self.model = YOLO(str(weights_path))
        try:
            self.model.fuse()
        except Exception:
            pass

        # Subscriber / Publisher
        self.st_Subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback_Image,
                                              queue_size=1, buff_size=2**24)
        self.st_Publisher  = rospy.Publisher("/YOLO/BBox", Yolo_bbox_arr, queue_size=5)

        self.st_BGRImg = None
        self.frame_seq = 0
        self.frame_lock = threading.Lock()
        self.rate = rospy.Rate(30)

        # --- 파라미터화 ---
        self.conf_thres = float(rospy.get_param("~conf_thres", 0.7))
        self.allowed_names = {s.strip().lower() for s in rospy.get_param("~allowed_names",
                                                                         ["bus","truck","person","car"])}

        # 모델 체크포인트에 들어있는 클래스명 사용 (id->name dict)
        self.names = self.model.names
        # 이름을 ID로 변환
        self.allowed_ids = {cid for cid, cname in self.names.items() if str(cname).lower() in self.allowed_names}
        if not self.allowed_ids:
            rospy.logwarn("[경고] allowed_ids가 비어 있음. 체크포인트 names 표기가 기대와 다를 수 있어, 루프에서 이름으로 재검증합니다.")

        self.SpeedSignDetection()

    def callback_Image(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img_bgr is not None:
                with self.frame_lock:
                    self.st_BGRImg = img_bgr
                    self.frame_seq += 1
        except Exception as e:
            rospy.logwarn(f"CompressedImage 디코드 실패: {e}")

    def SpeedSignDetection(self):
        while self.st_BGRImg is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        last_frame_seq = -1
        while not rospy.is_shutdown():
            with self.frame_lock:
                frame_seq = self.frame_seq
                img = None if self.st_BGRImg is None else self.st_BGRImg.copy()

            # Run inference once per newly received camera frame.
            if img is None or frame_seq == last_frame_seq:
                self.rate.sleep()
                continue
            last_frame_seq = frame_seq

            PublishMsg = Yolo_bbox_arr()
            count = 0

            infer_kwargs = dict(stream=True, verbose=False, conf=self.conf_thres)
            if self.allowed_ids:
                infer_kwargs["classes"] = list(self.allowed_ids)

            for result in self.model(img, **infer_kwargs):
                boxes = getattr(result, "boxes", None)
                if boxes is None or len(boxes) == 0:
                    continue

                for box in boxes:
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    name = self.names.get(cls_id, str(cls_id))

                    # allowed_ids가 비어 있으면 이름으로 최종 필터(체크포인트 표기 차이 대비)
                    if not self.allowed_ids and str(name).lower() not in self.allowed_names:
                        continue
                    if conf < self.conf_thres:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    PublishMsg.yolo_bbox_arr.append(
                        Yolo_bbox(xMin=x1, yMin=y1, xMax=x2, yMax=y2, classes=name)
                    )
                    count += 1

                    # 디스플레이가 필요하면 주석 해제
                    # cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 2)
                    # cv2.putText(img, f"{name} {conf:.2f}", (x1, max(0, y1-5)),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 1, cv2.LINE_AA)

            PublishMsg.objc = count
            self.st_Publisher.publish(PublishMsg)

            # 필요 시 미리보기
            # cv2.imshow("YOLO Detection", img); cv2.waitKey(1)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('YoloDetection', anonymous=True)
    st_LaneDetection()
    rospy.spin()
