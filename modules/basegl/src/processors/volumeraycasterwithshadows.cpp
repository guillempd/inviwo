/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2012-2021 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <modules/basegl/processors/volumeraycasterwithshadows.h>
#include <inviwo/core/io/serialization/serialization.h>
#include <inviwo/core/io/serialization/versionconverter.h>
#include <inviwo/core/interaction/events/keyboardevent.h>
#include <modules/opengl/image/layergl.h>
#include <modules/opengl/volume/volumegl.h>
#include <modules/opengl/texture/texture2d.h>
#include <modules/opengl/texture/textureunit.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/shader/shaderutils.h>
#include <modules/opengl/volume/volumeutils.h>
#include <inviwo/core/common/inviwoapplication.h>
#include <inviwo/core/util/rendercontext.h>
#include <inviwo/core/algorithm/boundingbox.h>

namespace inviwo {

    const ProcessorInfo VolumeRaycasterWithShadows::processorInfo_{
        "org.inviwo.VolumeRaycasterWithShadows",  // Class identifier
        "Volume Raycaster With Shadows",            // Display name
        "Volume Rendering",            // Category
        CodeState::Experimental,             // Code state
        "GL, DVR, Raycasting"          // Tags
    };

    VolumeRaycasterWithShadows::VolumeRaycasterWithShadows()
        : PoolProcessor()
        , shader_("raycastingwithshadows.frag", Shader::Build::No)
        , volumePort_("volume")
        , entryPort_("entry")
        , exitPort_("exit")
        , backgroundPort_("bg")
        , outport_("outport")
        , channel_("channel", "Render Channel", { {"Channel 1", "Channel 1", 0} }, 0)
        , raycasting_("raycaster", "Raycasting")
        , isotfComposite_("isotfComposite", "TF & Isovalues", &volumePort_,
            InvalidationLevel::InvalidResources)
        , camera_("camera", "Camera", util::boundingBox(volumePort_))
        , lighting_("lighting", "Lighting", &camera_)
        , positionIndicator_("positionindicator", "Position Indicator")
        , toggleShading_(
            "toggleShading", "Toggle Shading", [this](Event* e) { toggleShading(e); }, IvwKey::L)
        , enableSoftShadows_("enableSoftShadows", "Enable Soft Shadows", true, InvalidationLevel::InvalidResources)
        , lightDiameter_("lightDiameter", "Light Diameter", 1.0f, 0.1f, 10.0f)
        , opaqueThreshold_("opaqueThreshold", "Opaque Threshold", 0.99f, 0.1f, 1.0f)
        , translucentThreshold_("translucentThreshold", "Translucent Threshold", 0.15f, 0.1f, 1.0f)
        , softShadowsSamples_("softShadowsSamples", "Soft Shadows Samples", {1, 2, 4, 8, 16, 32}, 3) {

        shader_.onReload([this]() { invalidate(InvalidationLevel::InvalidResources); });

        addPort(volumePort_, "VolumePortGroup");
        addPort(entryPort_, "ImagePortGroup1");
        addPort(exitPort_, "ImagePortGroup1");
        addPort(outport_, "ImagePortGroup1");
        addPort(backgroundPort_, "ImagePortGroup1");

        backgroundPort_.setOptional(true);

        channel_.setSerializationMode(PropertySerializationMode::All);

        auto updateTFHistSel = [this]() {
            HistogramSelection selection{};
            selection[channel_] = true;
            isotfComposite_.setHistogramSelection(selection);
        };
        updateTFHistSel();
        channel_.onChange(updateTFHistSel);

        volumePort_.onChange([this]() {
            if (volumePort_.hasData()) {
                size_t channels = volumePort_.getData()->getDataFormat()->getComponents();

                if (channels == channel_.size()) return;

                std::vector<OptionPropertyIntOption> channelOptions;
                for (size_t i = 0; i < channels; i++) {
                    channelOptions.emplace_back("Channel " + toString(i + 1),
                        "Channel " + toString(i + 1), static_cast<int>(i));
                }
                channel_.replaceOptions(channelOptions);
                channel_.setCurrentStateAsDefault();
            }
            });
        backgroundPort_.onConnect([&]() { this->invalidate(InvalidationLevel::InvalidResources); });
        backgroundPort_.onDisconnect([&]() { this->invalidate(InvalidationLevel::InvalidResources); });

        // change the currently selected channel when a pre-computed gradient is selected
        raycasting_.gradientComputation_.onChange([this]() {
            if (channel_.size() == 4) {
                if (raycasting_.gradientComputation_.get() ==
                    RaycastingProperty::GradientComputation::PrecomputedXYZ) {
                    channel_.set(3);
                }
                else if (raycasting_.gradientComputation_.get() ==
                    RaycastingProperty::GradientComputation::PrecomputedYZW) {
                    channel_.set(0);
                }
            }
            });

        shader_.getFragmentShaderObject()->setShaderDefine("SOFT_SHADOWS_ENABLED", enableSoftShadows_);
        enableSoftShadows_.onChange([this]() {
            shader_.getFragmentShaderObject()->setShaderDefine("SOFT_SHADOWS_ENABLED", enableSoftShadows_);
            });

        addProperty(channel_);
        addProperty(raycasting_);
        addProperty(isotfComposite_);

        addProperty(camera_);
        addProperty(lighting_);
        addProperty(positionIndicator_);
        addProperty(toggleShading_);

        addProperty(enableSoftShadows_);
        addProperty(lightDiameter_);
        addProperty(opaqueThreshold_);
        addProperty(translucentThreshold_);
        addProperty(softShadowsSamples_);
    }

    const ProcessorInfo VolumeRaycasterWithShadows::getProcessorInfo() const { return processorInfo_; }

    void VolumeRaycasterWithShadows::initializeResources() {
        utilgl::addDefines(shader_, raycasting_, isotfComposite_, camera_, lighting_,
            positionIndicator_);
        utilgl::addShaderDefinesBGPort(shader_, backgroundPort_);
        shader_.build();
    }

    void VolumeRaycasterWithShadows::process() {
        if (volumePort_.isChanged()) {
            dispatchOne(
                [volume = volumePort_.getData()]() {
                volume->getRep<kind::GL>();
                glFinish();
                return volume;
            },
                [this](std::shared_ptr<const Volume> volume) {
                raycast(*volume);
                newResults();
            });
        }
        else {
            raycast(*volumePort_.getData());
        }
    }

    void VolumeRaycasterWithShadows::raycast(const Volume& volume) {
        if (!volume.getRep<kind::GL>()) {
            throw Exception("Could not find VolumeGL representation", IVW_CONTEXT);
        }
        utilgl::activateAndClearTarget(outport_);
        shader_.activate();

        TextureUnitContainer units;
        utilgl::bindAndSetUniforms(shader_, units, volume, "volume");
        utilgl::bindAndSetUniforms(shader_, units, isotfComposite_);
        utilgl::bindAndSetUniforms(shader_, units, entryPort_, ImageType::ColorDepthPicking);
        utilgl::bindAndSetUniforms(shader_, units, exitPort_, ImageType::ColorDepth);
        if (backgroundPort_.hasData()) {
            utilgl::bindAndSetUniforms(shader_, units, backgroundPort_, ImageType::ColorDepthPicking);
        }
        if (auto normals = entryPort_.getData()->getColorLayer(1)) {
            utilgl::bindAndSetUniforms(shader_, units,
                *normals->getRepresentation<LayerGL>()->getTexture(),
                std::string_view{ "entryNormal" });
            shader_.setUniform("useNormals", true);
        }
        else {
            shader_.setUniform("useNormals", false);
        }

        shader_.setUniform("lightRadius", lightDiameter_.get() / 2.0f);
        shader_.setUniform("opaqueThreshold", opaqueThreshold_.get());
        shader_.setUniform("translucentThreshold", translucentThreshold_.get());
        shader_.setUniform("softShadowsSamples", softShadowsSamples_.get());

        utilgl::setUniforms(shader_, outport_, camera_, lighting_, raycasting_, positionIndicator_,
            channel_, isotfComposite_);

        utilgl::singleDrawImagePlaneRect();

        shader_.deactivate();
        utilgl::deactivateCurrentTarget();
    }

    void VolumeRaycasterWithShadows::toggleShading(Event*) {
        if (lighting_.shadingMode_.get() == ShadingMode::None) {
            lighting_.shadingMode_.set(ShadingMode::Phong);
        }
        else {
            lighting_.shadingMode_.set(ShadingMode::None);
        }
    }

    // override to do member renaming.
    void VolumeRaycasterWithShadows::deserialize(Deserializer& d) {
        util::renamePort(d, { {&entryPort_, "entry-points"}, {&exitPort_, "exit-points"} });
        Processor::deserialize(d);
    }

}  // namespace inviwo
