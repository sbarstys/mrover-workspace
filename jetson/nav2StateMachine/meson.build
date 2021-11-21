project('nav2StateMachine', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')

executable('jetson_nav2StateMachine', 'pathFollowersBase.cpp', 'pathFollowersBase.hpp', 'simplePathFollower.hpp',
            'simplePathFollower.cpp', 'rover.hpp', 'rover.cpp', 'main.cpp',
           dependencies : [liblcm],
           install : true)