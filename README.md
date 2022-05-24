# suv_car

# 先编译ompl
```
cd src/planner/ompl/ompl-1.4.2_byxt

./build_ompl.sh
```

# 再单独编译mcp_msgs
```
cd suc_car

catkin_make -DCATKIN_WHITELIST_PACKAGES="mpc_msgs"
```

# 全局编译
```
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

# 生效
```
source suv_car/devel/setup.bash
```
