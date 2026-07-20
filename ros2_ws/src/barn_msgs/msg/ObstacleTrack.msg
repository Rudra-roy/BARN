# A single tracked dynamic obstacle, expressed in the array header's frame_id.
int32 id                          # stable track id (-1 = untracked detection)
geometry_msgs/Point position      # centroid position [m]
geometry_msgs/Vector3 velocity    # estimated velocity [m/s]
float64 radius                    # estimated obstacle radius [m]
float64 confidence                # 0..1, rises with track age / hit count
