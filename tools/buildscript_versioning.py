FILENAME_BUILDNO = '.pio/versioning'
FILENAME_VERSION_H = 'include/version.h'
version = 'v0.3.'

import datetime
from subprocess import *

build_no = 0
try:
    with open(FILENAME_BUILDNO) as f:
        build_no = int(f.readline()) + 1
except:
    print('Starting build number from 1..')
    build_no = 1
with open(FILENAME_BUILDNO, 'w+') as f:
    f.write(str(build_no))
    print('Build number: {}'.format(build_no))

version_full = version + str(build_no)

try:
    git_id = Popen('git rev-parse --short HEAD', stdout=PIPE, shell=True).stdout.read().strip().decode('ascii')
    version_full = "%s-%s" % (version_full, git_id)
except:
    pass

version_string = "{} - {}".format(version_full, datetime.datetime.now())
hf = """
#ifndef BUILD_NUMBER
  #define BUILD_NUMBER "{}"
#endif
#ifndef VERSION
  #define VERSION "{}"
#endif
#ifndef VERSION_SHORT
  #define VERSION_SHORT "{}"
#endif
""".format(build_no, version_string, version_full)
with open(FILENAME_VERSION_H, 'w+') as f:
    f.write(hf)

with open("data_embed/index.html", "r") as f:
    index_html_content = f.read()

index_html_content = index_html_content.replace('<!--VERSION-->', version_string)

with open("data_embed/index.html.out", "w") as f:
    f.write(index_html_content)
