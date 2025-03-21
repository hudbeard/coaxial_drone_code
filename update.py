import os
import subprocess


class Updator(object):
    def __init__(self):
        self.REPO_URL = "https://github.com/hudbeard/coaxial_drone_code.git"
        self.LOCAL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "repo")

    def download_project(self):
        if not os.path.exists(os.path.join(self.LOCAL_DIR, ".git")):
            subprocess.run(["git", "clone", self.REPO_URL, self.LOCAL_DIR], check=True)
        else:
            subprocess.run(["git", "-C", self.LOCAL_DIR, "pull"], check=True)

    @staticmethod
    def run_updated_code():
        from main import Drone
        drone = Drone()
        drone.run()


if __name__ == "__main__":
    updator = Updator()
    try:
        updator.download_project()
    except subprocess.CalledProcessError:
        print("Failed to update project...")

    updator.run_updated_code()
