if [ -f /usr/bin/clang-format-3.5 ]
then
	export CLANG_PATH="/usr/bin/clang-format-3.5"
fi

if [ -f /usr/bin/clang-format-3.6 ]
then
	export CLANG_PATH="/usr/bin/clang-format-3.6"
fi

if [ -f /usr/bin/clang-format-3.7 ]
then
  export CLANG_PATH="/usr/bin/clang-format-3.7"
fi

if [ -f /usr/bin/clang-format ]
then
	export CLANG_PATH="/usr/bin/clang-format"
fi

if [ -f /usr/local/bin/clang-format ]
then
  export CLANG_PATH="/usr/local/bin/clang-format"
fi

$CLANG_PATH -i $(git ls-files | grep "\(\.cpp\|\.h\)\$") -style=Google
