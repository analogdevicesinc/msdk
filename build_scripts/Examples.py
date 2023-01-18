#!/usr/bin/python

import os, sys
import ConfigParser


if len(sys.argv) < 2:
	print "Usage Error!"
	print "Usage:  ./Example.py device1 device2 device3...."
	sys.exit(0)


templates_folder="0_Templates"

print "----------------------------------------------------------------------"
print "Starting to prepare project related files from templates for all chips"
print "----------------------------------------------------------------------"


for chip in sys.argv[1:]:
	examples_path = "./Examples/" + chip

	print "Working for " + chip 

	for dirpath in os.listdir(examples_path):
		if not os.path.isdir(os.path.join(examples_path, dirpath)):
			continue

		folder=os.path.basename(dirpath)
		# pass templates
		if folder == templates_folder:
			continue

		config_file_path = os.path.join(examples_path, dirpath, 'example.cfg')
		if not os.path.isfile(config_file_path):
			print "\tWARNING:  no example.cfg for " + folder
			continue
		
		print "\tPreparing example files for : " + folder

		config = ConfigParser.RawConfigParser()
		config.read(config_file_path)

		# 
		# Create Makefile s 
		# 
		example_name = ""
		extra_src    = ""
		extra_ipath  = ""
		extra_vpath  = ""
		extra_def    = ""
		extra_mk     = ""
		extra_export = ""
		extra_cflag  = ""
		extra_libs   = ""
		extra_ldflag = ""
		extra_makecmds = ""
		optimization_level = ""
		ld_file = ""

		try:
			example_name = config.get('Makefile', 'name')
		except ConfigParser.NoOptionError:
			pass
		try:
			extra_src    = config.get('Makefile', 'extra_src')
		except ConfigParser.NoOptionError:
			pass
		try:
			extra_ipath  = config.get('Makefile', 'extra_ipath')
		except ConfigParser.NoOptionError:
			pass
		try:
			extra_vpath  = config.get('Makefile', 'extra_vpath')
		except ConfigParser.NoOptionError:
			pass
		try:
			extra_def    = config.get('Makefile', 'extra_def')
		except ConfigParser.NoOptionError:
			pass
		try:
			extra_mk     = config.get('Makefile', 'extra_mk')
		except ConfigParser.NoOptionError:
			pass
		try:
			extra_export     = config.get('Makefile', 'extra_export')
		except ConfigParser.NoOptionError:
			pass
		try:
			extra_cflag  = config.get('Makefile', 'extra_cflag')
		except ConfigParser.NoOptionError:
			pass
		try:
			extra_libs   = config.get('Makefile', 'extra_libs')
		except ConfigParser.NoOptionError:
			pass
		try:
			extra_ldflag = config.get('Makefile', 'extra_ldflag')
		except ConfigParser.NoOptionError:
			pass
		try:
			extra_makecmds = config.get('Makefile', 'extra_makecmds')
		except ConfigParser.NoOptionError:
			pass
		try:
			optimization_level = config.get('Makefile', 'optimization_level')
		except ConfigParser.NoOptionError:
			pass
		try:
			ld_file = config.get('Makefile', 'ld_file')
		except ConfigParser.NoOptionError:
			pass

		with open(os.path.join(examples_path, templates_folder, 'Makefile.tmp')) as infile, open(os.path.join(examples_path, dirpath, 'Makefile'), 'w') as outfile:
			for line in infile:
				if "##__EXTRA_SRC__##" in line:
					if len(extra_src) != 0:
						for item in extra_src.split(" "):
							if item != "":
								path = os.path.join(examples_path, dirpath, item)
								if os.path.isdir(path):
									for f in os.listdir(path):
										if f.endswith('.c') or f.endswith('.cpp'):
											outfile.write('SRCS += ' + item + '/' + f + '\n')
								else:
									outfile.write('SRCS += ' + item + '\n')
				elif "##__EXTRA_IPATH__##" in line:
					if len(extra_ipath) != 0:
						for item in extra_ipath.split(" "):
							if item != "":
								outfile.write('IPATH += ' + item + '\n')
				elif "##__EXTRA_VPATH__##" in line:
					if len(extra_vpath) != 0:
						for item in extra_vpath.split(" "):
							if item != "":
								outfile.write('VPATH += ' + item + '\n')
				elif "##__EXTRA_DEF__##" in line:
					if len(extra_def) != 0:
						for item in extra_def.split(" "):
							if item != "":
								outfile.write(item + '\n')
				elif "##__EXTRA_MK__##" in line:
					if len(extra_mk) != 0:
						for item in extra_mk.split(" "):
							if item != "":
								outfile.write('include ' + item + '\n')
				elif "##__EXTRA_EXPORT__##" in line:
					if len(extra_export) != 0:
						for item in extra_export.split(" "):
							if item != "":
								outfile.write('export ' + item + '\n')
				elif "##__EXTRA_CFLAG__##" in line:
					if len(extra_cflag) != 0:
						for item in extra_cflag.split(" "):
							if item != "":
								outfile.write('PROJ_CFLAGS += -D' + item + '\n')
				elif "##__EXTRA_LIBS__##" in line:
					if len(extra_libs) != 0:
						for item in extra_libs.split(" "):
							if item != "":
								outfile.write('PROJ_LIBS += ' + item + '\n')
				elif "##__EXTRA_LDFLAG__##" in line:
					if len(extra_ldflag) != 0:
						for item in extra_ldflag.split(" "):
							if item != "":							
								outfile.write('PROJ_LDFLAGS += -L' + item + '\n')
				elif "##__EXTRA_MAKECMDS__##" in line:
					if len(extra_makecmds) != 0:
						for idx,item in  enumerate(extra_makecmds.split(";")):
							if item != "":
								if idx%2 != 1:							
									outfile.write('\n' +item + ':\n')
								else:
									outfile.write('\t$(MAKE)  -f ' + item + '\n\n')
				elif "##__OPTIMIZATION_LEVEL__##" in line:
					if len(optimization_level) != 0:
						for item in optimization_level.split(" "):
							if item != "":
								outfile.write('MXC_OPTIMIZE_CFLAGS=' + item + '\n')
					else:
						outfile.write('#MXC_OPTIMIZE_CFLAGS=-O1' + '\n')

				elif "##__LINKER_FILE__##" in line:
					if len(ld_file) != 0:
						for item in ld_file.split(" "):
							if item != "":
								outfile.write( 'LINKER=' + item + '\n')
					else:
						outfile.write('LINKER=$(TARGET_LC).ld' + '\n')
				else:
					outfile.write(line)


		# 
		# Create Eclipse project files
		#

		# .project
		with open(os.path.join(examples_path, templates_folder, 'project.tmp')) as infile, open(os.path.join(examples_path, dirpath, '.project'), 'w+') as outfile:
			for line in infile:
				outfile.write(line.replace('##__PROJ_NAME__##', example_name))


		
		# .cproject
		eclipse_extra_ipath  = ""
		try:
			eclipse_extra_ipath = config.get('Eclipse-cproject', 'eclipse_extra_ipath')
		except ConfigParser.NoOptionError:
			pass

		with open(os.path.join(examples_path, templates_folder, 'cproject.tmp')) as infile, open(os.path.join(examples_path, dirpath, '.cproject'), 'w+') as outfile:
			for line in infile:
				if "##__ECLIPSE_EXTRA_IPATH__##" in line:
					if len(eclipse_extra_ipath) != 0:
						for item in eclipse_extra_ipath.split(" "):
							outfile.write('\t\t\t\t\t\t\t\t\t<listOptionValue builtIn=\"false\" value=\"&quot;' + item + '&quot;\"/>' + '\n')
				else:
					outfile.write(line.replace('##__PROJ_NAME__##', example_name))

		# .launch
		with open(os.path.join(examples_path, templates_folder, 'launch.tmp')) as infile, open(os.path.join(examples_path, dirpath, example_name + ".launch"), 'w+') as outfile:
			for line in infile:
				outfile.write(line.replace('##__PROJ_NAME__##', example_name))

		# # .settings
		# settings_path = os.path.join(examples_path, dirpath, ".settings")
		# if not os.path.isdir(settings_path) :
		# 	os.mkdir(settings_path)

		# with open(os.path.join(examples_path, templates_folder, 'settings.tmp')) as infile, open(os.path.join(settings_path, "language.settings.xml"), 'w+') as outfile:
		# 	for line in infile:
		# 		outfile.write(line.replace('##__PROJ_NAME__##', example_name))






