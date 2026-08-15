import cv2
import sys

class AVIPlayer:
    def __init__(self, file_path, target_fps=60):
        self.cap = cv2.VideoCapture(file_path)
        if not self.cap.isOpened():
            print(f"错误：无法打开视频文件 {file_path}")
            sys.exit(1)

        # 视频属性
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if self.total_frames <= 0:
            self.total_frames = None
            print("警告：无法获取总帧数，跳转时将无法验证范围")
        self.target_fps = target_fps
        self.wait_ms = int(1000 / target_fps)

        # 播放状态
        self.playing = True
        self.current_frame = 0   # 0-based
        self.window_name = "AVI Player - 空格暂停/播放，←/→跳帧，G跳转，Q退出"

        # 创建窗口并显示第一帧
        cv2.namedWindow(self.window_name)
        self._goto_frame(0)       # 跳到第0帧并显示

    def _goto_frame(self, frame_idx):
        """内部方法：跳转到指定帧（0-based）并刷新显示"""
        if self.total_frames is not None:
            frame_idx = max(0, min(self.total_frames - 1, frame_idx))
        else:
            frame_idx = max(0, frame_idx)

        self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = self.cap.read()
        if ret:
            self.current_frame = frame_idx
            self._update_display(frame)
        else:
            print(f"无法读取帧 {frame_idx}")

    def _update_display(self, frame):
        """更新窗口标题并显示图像"""
        status = "播放" if self.playing else "暂停"
        if self.total_frames is not None:
            title = f"{status} | 帧: {self.current_frame+1}/{self.total_frames} @ {self.target_fps} FPS"
        else:
            title = f"{status} | 帧索引: {self.current_frame} @ {self.target_fps} FPS"
        cv2.setWindowTitle(self.window_name, title)
        cv2.imshow(self.window_name, frame)

    def next_frame(self, step=1):
        new_frame = self.current_frame + step
        if self.total_frames is not None:
            new_frame = min(new_frame, self.total_frames - 1)
        self._goto_frame(new_frame)

    def prev_frame(self, step=1):
        new_frame = self.current_frame - step
        if self.total_frames is not None:
            new_frame = max(new_frame, 0)
        self._goto_frame(new_frame)

    def toggle_play(self):
        self.playing = not self.playing
        # 刷新标题（状态变化）
        ret, frame = self.cap.read()
        if ret:
            self._update_display(frame)
            # 注意：这里 read() 会前进一帧，但我们要保持当前位置，需重新设置
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.current_frame)
        else:
            # 若读取失败（例如视频结尾），重新定位
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.current_frame)
            _, frame = self.cap.read()
            if frame is not None:
                self._update_display(frame)

    def save_current_frame(self):
        """保存当前帧为图片"""
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.current_frame)
        ret, frame = self.cap.read()
        if ret:
            filename = f"frame_{self.current_frame+1}.png"
            cv2.imwrite(filename, frame)
            print(f"已保存: {filename}")
        else:
            print("无法保存当前帧")

    def run(self):
        """主循环"""
        while True:
            if self.playing:
                # 播放模式：读取下一帧
                ret, frame = self.cap.read()
                if not ret:
                    print("视频播放完毕，自动暂停")
                    self.playing = False
                    # 停在最后一帧
                    if self.total_frames:
                        self._goto_frame(self.total_frames - 1)
                    else:
                        # 未知总帧数时无法回退，直接退出循环
                        break
                    key = cv2.waitKey(0) & 0xFF
                    if not self._handle_key(key):
                        break
                    continue

                self.current_frame = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES)) - 1
                self._update_display(frame)
                key = cv2.waitKey(self.wait_ms) & 0xFF
            else:
                # 暂停模式：无限等待按键
                key = cv2.waitKey(0) & 0xFF

            if not self._handle_key(key):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def _handle_key(self, key):
        """处理按键，返回 False 表示退出循环"""
        if key == ord('q') or key == 27:          # q 或 ESC
            return False
        elif key == ord(' ') or key == ord('p'):  # 空格 或 p
            self.toggle_play()
        elif key == 81 or key == 2424832:         # 左箭头 ←
            self.prev_frame(step=10)
        elif key == 83 or key == 2555904:         # 右箭头 →
            self.next_frame(step=10)
        elif key == 82 or key == 2490368:         # 上箭头 ↑
            self.next_frame(step=30)
        elif key == 84 or key == 2621440:         # 下箭头 ↓
            self.prev_frame(step=30)
        elif key == ord('g'):                     # 跳转到指定帧（满足“选择某一帧”需求）
            if self.total_frames is not None:
                try:
                    target = input(f"请输入帧号 (1 - {self.total_frames}): ")
                    target_frame = int(target) - 1
                    if 0 <= target_frame < self.total_frames:
                        self._goto_frame(target_frame)
                        # 跳转后自动暂停，方便仔细查看
                        if self.playing:
                            self.toggle_play()
                    else:
                        print("帧号超出范围")
                except ValueError:
                    print("无效输入")
            else:
                print("无法跳转：总帧数未知（该视频不支持精确跳转）")
        elif key == ord('s'):                     # 保存当前帧
            self.save_current_frame()
        return True


def main():
    if len(sys.argv) > 1:
        video_path = sys.argv[1]
    else:
        video_path = "sample.avi"
    player = AVIPlayer(video_path, target_fps=60)
    player.run()


if __name__ == "__main__":
    main()
