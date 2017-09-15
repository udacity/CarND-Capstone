import cv2
import moviepy.editor as mpy
import os

class VideoMaker:
  def __init__(self, out_dir):
    self.out_dir = out_dir
    self.subclip_paths = []
    self.images = []
    self.frame_idx = 0

  def add_image(self, image):
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    self.images.append(rgb)
    self.frame_idx += 1
    if self.frame_idx % 500 == 0:
      self._make_subclip()

  def _make_subclip(self):
    subclip_path = os.path.join(self.out_dir, 'subclip_{}.mp4'.format(self.frame_idx))
    clip = mpy.ImageSequenceClip(self.images, fps=20)
    clip.write_videofile(subclip_path)
    self.subclip_paths.append(subclip_path)
    del self.images[:]

  def make_video(self, filename):
    self._make_subclip()
    final_path = os.path.join(self.out_dir, filename)
    _make_video(self.subclip_paths, final_path)

def _make_video(subclip_paths, filename):
  clips = []
  for subclip_path in subclip_paths:
    clips.append(mpy.VideoFileClip(subclip_path))
  final_clip = mpy.concatenate_videoclips(clips, method='compose')
  final_clip.write_videofile(filename)
  for subclip_path in subclip_paths:
    os.remove(subclip_path)
