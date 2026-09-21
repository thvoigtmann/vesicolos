# Local git setup

The VESICOLOS flight hardware usually does not have direct internet
connection; a typical setup is a local-network connection to a
ground-control computer (part of the EGSE).

We use a local bare clone of the git repository on the EGSE computer:
- on that computer, the github repository is checked out normally
- there, a second remote is configured,
  ```bash
  git remote add vlocal tv@localhost:$path_to_bare_clone/vesicolos.git
  ```
- the two copies are kept in sync by performing, in the normal code tree
  ```bash
  git fetch vlocal dev
  git merge vlocal/dev
  ```
  and similar push commands
- on the flight hardware, the `vlocal` repo is the main remote

Thus, if changes are made directly on the flight hardware, we need to
push them to the connected EGSE computer at some point. Then there,
we sync these changes back into the main tree, which we sync with github.

# Flight Hardware network

The VESCIOLOS Raspberry is normally hard-wired to the IP address
`192.168.100.12`, for the EGSE computer we use `192.168.100.42` (this
the VESICOLOS hardware only needs to know in case we want to sync
the git repo as described above).

How to setup fixed IP: something like
```bash
nmcli con add con-name vesicolos ifname enx00800f11732f type ethernet ip4 192.168.100.42/24
```
