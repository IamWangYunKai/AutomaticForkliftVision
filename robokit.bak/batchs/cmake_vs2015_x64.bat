@rem ************************************************
@rem * Script to cmake the solutions of robokit    *
@rem * Created by: Ye Yangsheng                    *
@rem * Created 2017.03.04                          *
@rem * © 2015-2017 Seer Robotics Co,.Ltd.          *
@rem * http://www.seer-robotics.com                *
@rem ************************************************

@echo ################################################################################
@echo #######################  RoboKit MSVC CMAKE Batch ##############################
@echo ###############  Copyright 2015-2017 Seer Robotics Co,.Ltd.  ###################
@echo ################################################################################
@echo cmake start ...
@if not exist ../build md ..\build
@cd ../build
@echo [%DATE% %Time%] Start cmake sequence
@rem Start cmake************************************************
@cmake -G "Visual Studio 14 2015 Win64" -DCMAKE_BUILD_TYPE=Relase ..
@if not %errorlevel% == 0 echo cmake failed! Error: %errorlevel%
@if %errorlevel% == 0 echo cmake successful
@echo [%DATE% %Time%] Finished cmake sequence
@rem sleep 3 seconds
@timeout /t 3
@cd ../batchs
