import os
from concurrent.futures import ThreadPoolExecutor

if __name__ == "__main__":
    py_dir = os.popen('which python3').read().strip()
    py_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "video_compressor.py")

    img_topic_list = [
        # "/head_camera/aligned_depth_to_color/image_raw", 
        "/head_camera/color/image_raw", 
        "/left_camera/color/image_raw", 
        "/right_camera/color/image_raw", 
    ]

    def do_something(topic_name):

        cmd_line = f"{py_dir} {py_path} --topic_name {topic_name}"
        print(cmd_line)
        os.system(cmd_line)

    with ThreadPoolExecutor(max_workers=4) as executor:
        executor.map(do_something, img_topic_list)

