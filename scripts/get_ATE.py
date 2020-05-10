#Original code
#https://github.com/MichaelGrupp/evo/blob/c50a0b34e61e92983d138941410d2afa0425a461/notebooks/metrics.py_API_Documentation.ipynb

import os
import sys
import glob
import pandas as pd

from evo.tools import file_interface
from evo.core import sync
from evo.core import trajectory
from evo.core import metrics
from evo.core.geometry import GeometryException

def compare_using_APE(ref_file, est_file, use_aligned_trajectories=False):
    """
    Compare two files using EVO API. Using the APE metric.
    :param ref_file:
    :param est_file:
    :param use_aligned_trajectories: True to align before comparing. False to leave original data as it is.
    :return:
    """
    # Load trajectories
    traj_ref = file_interface.read_tum_trajectory_file(ref_file)
    traj_est = file_interface.read_tum_trajectory_file(est_file)

    # Sinchronize trajectories by timestamps
    max_diff = 0.01
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff)

    # -------------EVO_APE-------------
    # Settings
    pose_relation = metrics.PoseRelation.translation_part
    use_aligned_trajectories = False  # OPTION -va on the scripts. Is related to Uleyamas alignment

    # The aligned trajectories can be used if we want it to
    if use_aligned_trajectories:
        # Align trajectories with Uleyamas algorithm
        try:
            traj_est_aligned = trajectory.align_trajectory(traj_est, traj_ref, correct_scale=False,
                                                           correct_only_scale=False)
            data = (traj_ref, traj_est_aligned)
        except GeometryException:
            print("Couldnt align with Uleyamas algorithm...")
            data = (traj_ref, traj_est)
    else:
        data = (traj_ref, traj_est)

    ape_metric = metrics.APE(pose_relation) # APE with only pose is in reality ATE (Absolute trajectory error) instead of APE (abs. pose error)
    ape_metric.process_data(data)

    # Get all stadistics in a dict
    ape_stats = ape_metric.get_all_statistics()
    return ape_stats


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Not enough arguments")
        exit(-1)

    groundtruth_path = sys.argv[1]
    estimation_path = sys.argv[2]

    if not os.path.exists(groundtruth_path):
        print("The grountruth path do not exist")
        exit(-1)

    if not os.path.exists(estimation_path):
        print("The estimation path do not exist")
        exit(-1)

        print(filepath)

    # APE OR ATE (SINCE IT ONLY USES THE TRANSLATION ERROR)
    print(compare_using_APE(ref_file=groundtruth_path, est_file=estimation_path, use_aligned_trajectories=False))


