import logging
from math import factorial
from itertools import permutations, islice

from resco_benchmark.config.config import config as cfg

logger = logging.getLogger(__name__)


def compute_safe_id(agent_id):
    total = len(cfg.uuid) + len(agent_id) + 8
    file_name_limit = 128
    safe_agt_id = agent_id.replace(":", "_")
    if total > file_name_limit:
        safe_agt_id = safe_agt_id[: len(agent_id) - (total - file_name_limit)]
    return safe_agt_id


def conv2d_size_out(size, kernel_size=2, stride=1):
    return (size - (kernel_size - 1) - 1) // stride + 1


def one_hot_list(signal):
    acts = signal.green_phases
    one_hotted = [0] * len(acts)
    one_hotted[signal.current_phase] = 1
    return one_hotted


def permutations_without_rotations(lst):
    return islice(permutations(lst), factorial(max(len(lst) - 1, 0)))


def cleanup_log_dir():
    # Remove signals.pkl, state.xml.gz, metrics_*.csv, tripinfo_*.xml from THIS run's folder only.
    # It used to walk all of cfg.log_dir, so a job finishing in parallel deleted the live
    # state/tripinfo files of every other job writing to the same log_dir (crashing Salt Lake
    # runs mid-episode and dropping episodes from Cologne1 runs).
    import os
    import re

    for root, dirs, files in os.walk(cfg.run_path):
        for file in files:
            if file == "signals.pkl" or file == "state.xml.gz":
                try:
                    os.remove(os.path.join(root, file))
                except Exception as e:
                    logger.error("Failed to remove {0}: {1}".format(file, e))
            elif re.match(r"metrics_\d+\.csv", file) or re.match(
                r"tripinfo_\d+\.xml", file
            ):
                try:
                    os.remove(os.path.join(root, file))
                except Exception as e:
                    logger.error("Failed to remove {0}: {1}".format(file, e))


def unpack_slurm_folders(path):
    import os
    import shutil

    folders = [f for f in os.listdir(path) if f.startswith("SLURM")]
    for folder in folders:
        full_path = os.path.join(path, folder)
        for item in os.listdir(full_path):
            s = os.path.join(full_path, item)
            d = os.path.join(path, item)
            if os.path.isdir(s):
                shutil.copytree(s, d, dirs_exist_ok=True)
            else:
                shutil.copy2(s, d)
        shutil.rmtree(full_path)


if __name__ == "__main__":
    print("You may want to backup first in case anything is overwritten")
    unpack_slurm_folders(cfg.log_dir)
    # cleanup_log_dir()
