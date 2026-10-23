"""Compatibility entrypoint: live worksheet content comes from the curriculum."""
from build_notebooks import main, notebook
from workshop.curriculum import load_curriculum

def build():
    return notebook(load_curriculum(), live=True)

if __name__ == '__main__': main()
