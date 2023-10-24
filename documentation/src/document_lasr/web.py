import document_lasr
import shutil
import rospkg
import os

r = rospkg.RosPack()

def configure_web():
    # Determine dest folder
    PKG = r.get_path('documentation')
    WEB_DIR = os.path.abspath(os.path.join(PKG, 'web'))
    DOC_DIR = os.path.join(WEB_DIR, 'docs', 'packages')
    TASK_DIR = os.path.join(WEB_DIR, 'docs', 'tasks')

    for DIR in [DOC_DIR, TASK_DIR]:
        # Clear files
        if os.path.exists(DIR):
            shutil.rmtree(DIR)

        # Make the directory
        os.mkdir(DIR)

    # Copy README files
    for pkg in document_lasr.pkg_lasr_list():
        README = os.path.join(pkg, 'README.md')
        if os.path.exists(README):
            shutil.copyfile(README, os.path.join(TASK_DIR if '/tasks/' in pkg else DOC_DIR, os.path.basename(pkg) + '.md'))

    # Write category file for packages
    with open(os.path.join(DOC_DIR, '_category_.json'), 'w') as f:
        f.write('{"label":"Packages","position":4}')
    
    with open(os.path.join(TASK_DIR, '_category_.json'), 'w') as f:
        f.write('{"label":"Tasks","position":3}')

    # Change directory
    os.chdir(WEB_DIR)
