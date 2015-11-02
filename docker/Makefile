all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build            - build all cogrob images"
	@echo "   1. make pull             - pull all cogrob images"
	@echo "   1. make clean            - remove all cogrob images"
	@echo ""

build:
	@docker build --tag=cogrob/object_tracking_2d_ros:dep dep/.
	@docker build --tag=cogrob/object_tracking_2d_ros:dev dev/.
	@docker build --tag=cogrob/object_tracking_2d_ros:ocv ocv/.
	@docker build --tag=cogrob/object_tracking_2d_ros:ebt ebt/.
	@docker build --tag=cogrob/object_tracking_2d_ros:gui gui/.
	@docker build --tag=cogrob/object_tracking_2d_ros:nvidia nvidia/.

# pull:
# 	@docker pull cogrob/object_tracking_2d_ros:dep

# clean:
# 	@docker rmi -f cogrob/object_tracking_2d_ros:dep
# 	@docker rmi -f cogrob/object_tracking_2d_ros:dev
# 	@docker rmi -f cogrob/object_tracking_2d_ros:ocv
# 	@docker rmi -f cogrob/object_tracking_2d_ros:ebt
# 	@docker rmi -f cogrob/object_tracking_2d_ros:gui
# 	@docker rmi -f cogrob/object_tracking_2d_ros:nvidia
