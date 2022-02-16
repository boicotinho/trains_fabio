project("common")

find_package(Boost 1.66 COMPONENTS)
find_package(Threads REQUIRED)

#if(Boost_FOUND)
    #include_directories(${Boost_INCLUDE_DIRS} SYSTEM)
    #add_executable(progname file1.cxx file2.cxx)
    #target_link_libraries(progname ${Boost_LIBRARIES})
#endif()

file (GLOB SOURCE_FILES *.cpp)
file (GLOB HEADER_FILES *.h)

add_library( ${PROJECT_NAME} ${SOURCE_FILES} )

target_include_directories( ${PROJECT_NAME} SYSTEM INTERFACE ../)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC Threads::Threads)
