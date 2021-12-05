project('nav2StateMachine', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')


executable('jetson_nav2StateMachine',  'main.cpp', 
            'pathFollowerBase.cpp', 
            'simplePathFollower.cpp',
            'rover.cpp',
            'pid.cpp', 
            'utilities.cpp',
            'gimbal.cpp',
           dependencies : [liblcm], 
           link_args : '-ldl',
           install : true)