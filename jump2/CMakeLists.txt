# create an executable
ADD_EXECUTABLE (jump2 main.cpp
	${PROJECT_SOURCE_DIR}/src/system_model/SimulationSystemModel.cpp
	ToroJumpController.cpp
	)

# and link the library against the executable
TARGET_LINK_LIBRARIES (jump2 ${COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/jump2)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})