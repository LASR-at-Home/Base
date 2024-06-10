#!/usr/bin/env python3
import document_lasr

# Document everything
for pkg in document_lasr.pkg_lasr_list():
    print(pkg)
    document_lasr.generate_readme(pkg)
