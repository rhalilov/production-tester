# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(stepper_f407_emul)

target_sources(app PRIVATE
	src/main.c
	src/io.c
	src/motion.c
	src/safety.c
	src/home.c
	src/smema.c
	src/solenoid.c
	src/evlog.c
	src/process.c
	src/roles.c
	src/cli.c
)
