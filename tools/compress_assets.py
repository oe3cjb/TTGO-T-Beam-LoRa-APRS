import gzip
assets_list = {
    'data_embed/index.html.out': 'data_embed/index.html.out',
    'data_embed/js.js': 'data_embed/js.js.out',
    'data_embed/style.css': 'data_embed/style.css.out',
}


for src_file_name, out_file_name in assets_list.items():
    with open(src_file_name, 'rb') as f:
        content = f.read()
    with open(out_file_name, 'wb') as f:
        f.write(gzip.compress(content, compresslevel=9))
