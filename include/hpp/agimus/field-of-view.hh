// Copyright 2022, CNRS, Airbus SAS

// Author: Diane Bury

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided
// with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef HPP_AGIMUS_FIELD_OF_VIEW_HH
#define HPP_AGIMUS_FIELD_OF_VIEW_HH

#include <hpp/util/pointer.hh>
#include <hpp/agimus/fwd.hh>
#include <hpp/fcl/data_types.h>


namespace hpp {
  namespace agimus {
    using hpp::fcl::Triangle;

    typedef std::vector<Triangle> Tetahedron;

    struct Feature
    {
      Feature(std::string& name, value_type size):
        name(name), size(size) {}

      std::string name;
      value_type size;
    }; // struct Feature


    struct FeatureGroup
    {
      std::vector<Feature> features;
      int n_visibility_threshold;
      value_type depth_margin;
      value_type size_margin;
      FeatureGroup(int n_visibility_thr, value_type depth_margin,
                   value_type size_margin)
          : n_visibility_thr(n_visibility_thr),
          depth_margin(depth_margin),
          size_margin(size_margin)
        {};
    }; // struct FeatureGroup
    typedef std::make_shared<FeatureGroup> FeatureGroupPtr_t;
    typedef std::vector <FeatureGroupPtr_t> FeatureGroups_t;


    class FieldOfView
    {
    public:
      static FieldOfViewPtr_t create(const ProblemSolverPtr_t& ps)
      {
        FieldOfViewPtr_t ptr (new FieldOfView(ps));
        ptr->init(ptr);
        return ptr;
      }
      int numberVisibleFeature(const FeatureGroupPtr_t& fg);
      bool clogged();

      void addFeatureGroup(const FeatureGroupPtr_t& fg)
      {
        featureGroups_.push_back(fg);
      }

      void resetFeatureGroups()
      {
        featureGroups_.clear();
      }

    private:
      // Constructor
      FieldOfView(const ProblemSolverPtr_t& ps);
      void init(const FieldOfViewWkPtr_t)
      {}

      void featureToTetahedrontPts();
      bool featureVisible();
      bool robotClogsFieldOfView();

      ProblemSolverPtr_t problemSolver_;
      bool display_;
      FeatureGroups_t featureGroups_;

    }; // class FieldOfView
  } // namespace agimus
} // namespace hpp

#endif // HPP_AGIMUS_FIELD_OF_VIEW_HH