cartan
=======

c++ geometric traits and algorithms library.

### Why ###

The rationale behind this library is essentially the same as behind the boost graph library - just for polygonal mesh data and pointclouds.
Once a simple, traits-based access to primitive operations on geometry is possible, any algorithms based on these are independent of the actual representation of the geometry.

### For Whom ###

Me. This is far from being a generally usable library and is supposed to be developed by me whenever I fix it or need new features.
If you by any chance happen to need this library or want to contribute, you should really only consider doing so if
- you have no problem at all working with template traits classes (it gets messy, believe me)
- you have no problem at all with missing/wrong documentation
- you realize that such a generic approach is actually necessary in your case (in almost all cases just sticking with e.g. OpenMesh should suffice)


### For What ###

The original purpose of this library is to be used by "some folks" at the University of Bonn, specifically for the harmont library and graphene tool (which can be found at github.com/paulhilbert).
