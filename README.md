# TITAN DOCUMENTATION

Static Site Generator Used: [MKDocs](https://www.mkdocs.org/)

As MKDocs requires older python package versions, we use a virtual environment

## Setup
To create a virtual environment
```bash
python3 -m venv ~/mkdocs-venv
source ~/mkdocs-venv/bin/activate
```
Upgrade pip and install MKDocs
```bash
pip install --upgrade pip
pip install mkdocs mkdocs-material
```

## Using MKDocs
To use MKDocs:

Source the environment
```bash
source ~/mkdocs-venv/bin/activate
```

To create a new project
```bash
mkdocs new my-project
cd my-project
```

You will notice that it generate 2 directories (docs and site) and a .yml file

The ```docs``` directory has the markdown files

The ```site``` directory auto-generates the html and css part when you build

The ```mkdocs.yml``` file is used to arrange the order of the markdown files

Now to build the website

```bash
mkdocs build
```

To run the website
```
mkdocs serve
```

Ctrl + click on the static link and view the site.

## Using Github pages

Ensure that your mkdocs workspace is pushed to a github repo. Then in the terminal:
```bash
mkdocs gh-deploy
```
This pushes the workspace as a github page. In your repo, go to Settings->Pages and start the Github Pages. Your Website is permanently running.

Note: .md files support html
