#!/usr/bin/env python3
"""Stable project wrapper around evo_ape/evo_rpe/evo_traj."""
import argparse, subprocess

def main():
    p=argparse.ArgumentParser(); p.add_argument("mode",choices=["ape","rpe","traj"]);p.add_argument("format",choices=["kitti","tum","euroc"]);p.add_argument("reference");p.add_argument("estimates",nargs='+');p.add_argument("--align",choices=["none","origin","se3","sim3"],default="origin");p.add_argument("--plot",action="store_true");p.add_argument("--save-results");p.add_argument("--delta",type=float,default=1.0);a=p.parse_args()
    cmd=[f"evo_{a.mode}",a.format]
    if a.mode=="traj": cmd += a.estimates+["--ref",a.reference]
    else: cmd += [a.reference,a.estimates[0],"-r","full"]
    cmd += {"none":[],"origin":["--align_origin"],"se3":["--align"],"sim3":["--align","--correct_scale"]}[a.align]
    if a.mode=="rpe":cmd += ["--delta",str(a.delta)]
    if a.plot:cmd.append("--plot" if a.mode!="traj" else "-p")
    if a.save_results and a.mode!="traj":cmd += ["--save_results",a.save_results]
    subprocess.run(cmd,check=True)
if __name__=="__main__":main()
