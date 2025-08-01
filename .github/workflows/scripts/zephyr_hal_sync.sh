#!/bin/bash

hal_adi_branch="msdk-export"

if [ $# -eq 2 ]; then
    msdk="$1"
    hal_adi="$2"
else
    msdk="./msdk"
    hal_adi="./hal_adi"
fi

root_dir=$(pwd)

hal_adi_current_msdk_sha=""

############################################################################################################

pushd ${hal_adi}

git checkout ${hal_adi_branch} 2>&1 > /dev/null

if [ $? -ne 0 ]
then
	echo
	echo "Creating an empty branch in hal_adi repository..."
	echo

	git checkout --orphan ${hal_adi_branch}
	git rm -r --cached .
	rm -rf ./*
	git checkout origin/develop -- CMakeLists.txt
	git checkout origin/develop -- MAX/CMakeLists.txt
	git checkout origin/develop -- LICENSE.md
	git checkout origin/develop -- README.md
	git checkout origin/develop -- zephyr/module.yml
else

	iterator=0
	while [ -z "${hal_adi_current_msdk_sha}" ]
	do
		hal_adi_current_msdk_sha=$(git show --format="%(trailers:key=MSDK-Commit,valueonly,separator=%x2C)" --no-patch "HEAD~${iterator}")
		iterator=$(( $iterator + 1 ))
	done
	echo
	echo "Adding changes since ${hal_adi_current_msdk_sha} to the existing branch"
	echo
fi

popd


############################################################################################################
echo
echo "Getting all commits of MSDK repository..."
echo

pushd ${msdk}

git fetch origin
git rev-list --reverse --topo-order origin/main > ${root_dir}/git_sha_log_reverse.txt

if [ -n "${hal_adi_current_msdk_sha}" ]
then
	echo "Trimming list to start after ${hal_adi_current_msdk_sha}"
	sed -i -e "1,/${hal_adi_current_msdk_sha}/d" ${root_dir}/git_sha_log_reverse.txt
fi

num_of_sha=$(wc -l ${root_dir}/git_sha_log_reverse.txt | awk '{print $1}')

if [ "${num_of_sha}" -eq "1" ]; then
	one_commit=$(head ${root_dir}/git_sha_log_reverse.txt)
	if [ "${one_commit}" = "${hal_adi_current_msdk_sha}" ]; then
		echo "Nothing to do, no new commits after ${hal_adi_current_msdk_sha}"
		exit 0
	fi
fi

popd

############################################################################################################
echo
echo "Starting to create/update history of hal_adi..."

check_is_adi_hal_lib () {
  [ "x$1" = x ] && return 1
  for pf in Libraries/CMSIS/Include              \
            Libraries/CMSIS/Device              \
            Libraries/PeriphDrivers/Source              \
            Libraries/PeriphDrivers/Include              \
            Libraries/MAXUSB/include/core              \
            Libraries/MAXUSB/src/core; do
      suf="${1#$pf}"
      [ x"$pf$suf" = x"$1" ] && return 0
  done
  return 1
}

check_is_adi_hal_zephyr_file () {
  [ "x$1" = x ] && return 1
  for pf in Libraries/zephyr/MAX; do
      suf="${1#$pf}"
      [ x"$pf$suf" = x"$1" ] && return 0
  done
  return 1
}

for ((index=1; index<=num_of_sha; index++)); do
    echo
    echo "Processing commit $index of $num_of_sha"

    current_sha=$(sed -n "${index}p" ${root_dir}/git_sha_log_reverse.txt)

    pushd ${msdk}

    git show --pretty="" --name-only ${current_sha} > ${root_dir}/current_changed_files.txt
    commit_author=$(git show --no-patch --format="%an <%ae>" ${current_sha})
    commit_msg=$(git show --no-patch --format="%B" ${current_sha})
    commit_date=$(git show --no-patch --format="%aI" ${current_sha})

    git checkout -f $current_sha

    popd


    while read NAME
    do
	if check_is_adi_hal_lib "$NAME"; then
		dest="${hal_adi}/MAX/${NAME}"
		mkdir -p $(dirname "${dest}")
		if [ "${NAME##*.}" = "svd" ]; then
			echo "Skipping ignored suffix svd: ${NAME}"
		elif [ -e "${msdk}/${NAME}" ]; then
			cp "${msdk}/${NAME}" "${dest}"
			# Remove a few file types we don't want to include, to be safe
			rm -rf ${hal_adi}/MAX/Libraries/CMSIS/Device/Maxim/GCC
			rm -rf ${hal_adi}/MAX/Libraries/CMSIS/Device/Maxim/MAX*/Source/IAR
			rm -rf ${hal_adi}/MAX/Libraries/CMSIS/Device/Maxim/MAX*/Source/GCC
			rm -rf ${hal_adi}/MAX/Libraries/CMSIS/Device/Maxim/MAX*/Source/ARM
			rm -rf ${hal_adi}/MAX/Libraries/MAXUSB/include/core/arm
			rm -rf ${hal_adi}/MAX/Libraries/MAXUSB/include/core/maxq
			rm -rf ${hal_adi}/MAX/Libraries/MAXUSB/src/core/arm
			rm -rf ${hal_adi}/MAX/Libraries/MAXUSB/src/core/maxq
		else
			rm "${dest}"
		fi

	elif check_is_adi_hal_zephyr_file "$NAME"; then
		dest="${hal_adi}/${NAME##Libraries/zephyr/}"
		echo "New dest for Zephyr file is ${dest}"
		mkdir -p $(dirname "${dest}")
		cp "${msdk}/${NAME}" "${dest}"

	fi
    done < ${root_dir}/current_changed_files.txt

    pushd ${hal_adi}

    if [ -n "$(git status -s)" ]; then
        echo "New change exist need to be committed to hal_adi!"
        # echo "${current_sha}" > MAX/msdk_sha
        git add *
        git commit --author="${commit_author}" --date="${commit_date}" -m "${commit_msg}" --trailer "MSDK-Commit: ${current_sha}"
    else
        echo "No need to commits to hal_adi"
    fi;

    popd
done
