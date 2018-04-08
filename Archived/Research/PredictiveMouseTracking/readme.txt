Workflow:

1. Run driver.py (in xwindows) to gather users' time series files into the
   data subdirectory.

   (Out: data/
    reach*, traj*)


2. Run trialconcat.py to gather the trials of each task type (reach, traj)
   into one large CSV file.

   Ex:
     ./trialconcat.py 0.025 processed_data/reach_keys.csv processed_data/reach.csv data/reach*

   (Out: processed_data/
    reach.csv, reach_keys.csv, traj.csv, traj_keys.csv)


3. Run process.py to do basic processing on this file -- generate mirrored
   trials, transform into relative coordinates, whiten.

   Ex:
      ./process.py reach
      ./process.py traj

   (Out: processed_data/
    reach_scaled.csv, reach_trans_scaled.csv, reach-mean-stdev.csv
    traj_scaled.csv, traj_trans_scaled.csv, traj-mean-stdev.csv)


4. Run makehist.py to generate history variables for the autoregression.

   Ex:
     ./makehist.py processed_data/reach_scaled.csv processed_data/reach_h3.csv "widget dx[-1]" "widget dx[-2]" "widget dx[-3]" "widget dy[-1]" "widget dy[-2]" "widget dy[-3]"

   (Out: processed_data/
    reach_hX.csv, reach_trans_hX.csv, traj_hX.csv, traj_trans_hX.csv)

4. Run process.py to drop the 'trial' variable from those files.

   Ex:
     ./process.py processed_data/reach_hX.csv processed_data/reach_hX.csv delete_key trial


5. Run gmmfit on the files to generate transition and observation models.

   ./gmmfit processed_data/reach_h3.csv 5 500 models/reach_obs5_h3.gmm

   (Out: models/
    reach_obsM_hX.gmm, reach_transN_hX.gmm,
    traj_obsM_hX.gmm, traj_transN_hX.gmm)


6. Cross-validate the fitted data, select the best values of X, M, N.  [TODO]


7. Make an XML file describing the best model.

