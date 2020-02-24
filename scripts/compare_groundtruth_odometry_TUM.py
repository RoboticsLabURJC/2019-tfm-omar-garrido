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

    ape_metric = metrics.APE(pose_relation)
    ape_metric.process_data(data)

    # Get all stadistics in a dict
    ape_stats = ape_metric.get_all_statistics()
    return ape_stats

def compare_using_RPE(ref_file, est_file, use_aligned_trajectories=False):
    # Load trajectories
    traj_ref = file_interface.read_tum_trajectory_file(ref_file)
    traj_est = file_interface.read_tum_trajectory_file(est_file)

    # Sinchronize trajectories by timestamps
    max_diff = 0.01
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff)

    # Settings
    pose_relation = metrics.PoseRelation.full_transformation

    # normal mode
    delta = 1
    delta_unit = metrics.Unit.frames

    # all pairs mode
    all_pairs = False  # activate

    data = (traj_ref, traj_est)

    # -------------EVO_RPE-------------
    rpe_metric = metrics.RPE(pose_relation, delta, delta_unit, all_pairs)
    rpe_metric.process_data(data)

    rpe_stats = rpe_metric.get_all_statistics()
    return rpe_stats

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Not enough arguments")
        exit(-1)

    groundtruth_path = sys.argv[1]
    estimations_path = sys.argv[2]

    if not os.path.isdir(groundtruth_path):
        print("The grountruth path do not exist")
        exit(-1)

    if not os.path.isdir(estimations_path):
        print("The grountruth path do not exist")
        exit(-1)

    # Get all files within each directory
    groundtruth_file_list = glob.glob(os.path.join(groundtruth_path, "*"))
    estimations_file_list = glob.glob(os.path.join(estimations_path, "*"))

    # Output file
    output_ape_path = "ape_results.csv"
    output_rpe_path = "rpe_results.csv"
    # The name of each file will represent the index of the dataframe
    indexes = []
    ape_data = []
    rpe_data = []

    # For each groundtruth file look for the estimated file, compare and save the results
    for filepath in groundtruth_file_list:
        filename = os.path.basename(filepath)
        dataset_name = filename[:22]
        sequence_name = filename[23:-16]

        substring = "f" + dataset_name[-1] + "_" + sequence_name + "-"  # Since there are desk and desk2 I also need to seach for the "-" to indicate it has finish
        found = False

        print(filepath)

        for estimationpath in estimations_file_list:
            if substring in estimationpath:
                # Both files groundtruth and estimation are located
                found = True

                ape_data.append(compare_using_APE(ref_file=filepath, est_file=estimationpath, use_aligned_trajectories=False))
                rpe_data.append(compare_using_RPE(ref_file=filepath, est_file=estimationpath, use_aligned_trajectories=False))
                indexes.append(filename)

        if found is False:
            print("ERROR: The file " + filename + " couldnt not found and estimated file ")

    # Create dataframes and output files
    ape_df = pd.DataFrame(ape_data, index=indexes)
    rpe_df = pd.DataFrame(rpe_data, index=indexes)
    ape_df.to_csv(output_ape_path)
    rpe_df.to_csv(output_rpe_path)
