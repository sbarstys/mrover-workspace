project('nav2StateMachine', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')
follower = include_directories('followers')

executable('jetson_nav2StateMachine', 'followers/pathFollowerBase.cpp', 'followers/pathFollowerBase.hpp', 
            'followers/simplePathFollower.hpp', 'followers/simplePathFollower.cpp', 'rover.hpp', 'rover.cpp', 'main.cpp', 'path.hpp',
            'pid.hpp', 'pid.cpp', 'utilities.cpp', 'utilities.hpp',
           dependencies : [liblcm], link_args : '-ldl',
           install : true)