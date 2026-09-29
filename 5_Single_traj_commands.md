## Relative
For right_j0 +π/2:

python3 single_joint_trajectory.py \
    --joint right_j0 \
    --relative 1.57079632679 \
    --time 5

For right_j0 −π/2:
python3 single_joint_trajectory.py \
    --joint right_j0 \
    --relative -1.57079632679 \
    --time 5

For another joint, such as right_j3 +π/2:
python3 single_joint_trajectory.py \
    --joint right_j3 \
    --relative 1.57079632679 \
    --time 5

For an absolute target, for example move right_j2 to exactly 1.0 rad:
python3 single_joint_trajectory.py \
    --joint right_j2 \
    --absolute 1.0 \
    --time 5

Or move right_j5 to:
-0.8 rad

with:
python3 single_joint_trajectory.py \
    --joint right_j5 \
    --absolute -0.8 \
    --time 5