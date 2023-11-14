To re-build the gentoo_prefix_32b image with current gentoo snapshot, please run the following:

```bash
docker build --progress plain -f Dockerfile.prefix -t gentoo_prefix_32b .
```

Then, you can compress the archive using the local image:

```bash
docker run --entrypoint /tmp/gentoo/executeonprefix gentoo_prefix_32b  "tar -c --lzma -f - -C /tmp gentoo" > ~/gentoo_on_tmp.tar.lzma
```