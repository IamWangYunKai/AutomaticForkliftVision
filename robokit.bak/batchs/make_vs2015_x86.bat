@rem ************************************************
@rem * Script to compile the solutions of robokit  *
@rem * Created by: Ye Yangsheng                    *
@rem * Created 2017.03.04                          *
@rem * © 2015-2017 Seer Robotics Co,.Ltd.          *
@rem * http://www.seer-robotics.com                *
@rem ************************************************

@echo ################################################################################
@echo #######################  RoboKit MSVC Make Batch ###############################
@echo ###############  Copyright 2015-2017 Seer Robotics Co,.Ltd.  ###################
@echo ################################################################################
@echo cmake ^& build start ...
@call cmake_vs2015_x86.bat
@if %errorlevel% == 0 call build_vs2015_x86.bat
