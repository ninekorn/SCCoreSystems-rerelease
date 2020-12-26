﻿//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     Runtime Version:4.0.30319.42000
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

namespace SCCoreSystems.Properties {
    using System;
    
    
    /// <summary>
    ///   A strongly-typed resource class, for looking up localized strings, etc.
    /// </summary>
    // This class was auto-generated by the StronglyTypedResourceBuilder
    // class via a tool like ResGen or Visual Studio.
    // To add or remove a member, edit your .ResX file then rerun ResGen
    // with the /str option, or rebuild your VS project.
    [global::System.CodeDom.Compiler.GeneratedCodeAttribute("System.Resources.Tools.StronglyTypedResourceBuilder", "15.0.0.0")]
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
    [global::System.Runtime.CompilerServices.CompilerGeneratedAttribute()]
    internal class Resources {
        
        private static global::System.Resources.ResourceManager resourceMan;
        
        private static global::System.Globalization.CultureInfo resourceCulture;
        
        [global::System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal Resources() {
        }
        
        /// <summary>
        ///   Returns the cached ResourceManager instance used by this class.
        /// </summary>
        [global::System.ComponentModel.EditorBrowsableAttribute(global::System.ComponentModel.EditorBrowsableState.Advanced)]
        internal static global::System.Resources.ResourceManager ResourceManager {
            get {
                if (object.ReferenceEquals(resourceMan, null)) {
                    global::System.Resources.ResourceManager temp = new global::System.Resources.ResourceManager("SCCoreSystems.Properties.Resources", typeof(Resources).Assembly);
                    resourceMan = temp;
                }
                return resourceMan;
            }
        }
        
        /// <summary>
        ///   Overrides the current thread's CurrentUICulture property for all
        ///   resource lookups using this strongly typed resource class.
        /// </summary>
        [global::System.ComponentModel.EditorBrowsableAttribute(global::System.ComponentModel.EditorBrowsableState.Advanced)]
        internal static global::System.Globalization.CultureInfo Culture {
            get {
                return resourceCulture;
            }
            set {
                resourceCulture = value;
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to //////////////////////
        ///////   TYPES
        /////////////////////////
        ///struct PixelInputType
        ///{
        ///	float4 position : SV_POSITION;
        ///	float4 color : COLOR;
        ///};
        ///
        /////////////////////////
        ///////   Pixel Shader
        ////////////////////////
        ///float4 ColorPixelShader(PixelInputType input) : SV_TARGET
        ///{
        ///	// EX: 5 - Change pixel shader output to half brightness.
        ///	//input.color.g *= 0.5f;
        ///	return input.color;
        ///}
        ///.
        /// </summary>
        internal static string Color {
            get {
                return ResourceManager.GetString("Color", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to ///////////////////////
        ///////   GLOBALS
        //////////////////////////
        /////float4x4 worldMatrix;
        /////float4x4 viewMatrix;
        /////float4x4 projectionMatrix;
        ///
        ///cbuffer MatrixBuffer :register(b0)
        ///{
        ///	float4x4 worldMatrix;
        ///	float4x4 viewMatrix;
        ///	float4x4 projectionMatrix;
        ///}
        ///
        /////////////////////////
        ///////   TYPES
        /////////////////////////
        ///struct VertexInputType
        ///{
        ///	float4 position : POSITION;
        ///	float4 color : COLOR;
        ///};
        ///
        ///struct PixelInputType
        ///{
        ///	float4 position : SV_POSITION;
        ///	float4 color : COLOR;
        ///};
        ///
        ////////// [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string Color1 {
            get {
                return ResourceManager.GetString("Color1", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to ////////////////////////////////////////////////////////////////////////////////
        ///// Filename: font.ps
        ///////////////////////////////////////////////////////////////////////////////////
        ///
        ///
        ////////////////
        ///// GLOBALS //
        ////////////////
        ///Texture2D shaderTexture;
        ///SamplerState SampleType;
        ///
        ///cbuffer PixelBuffer
        ///{
        ///    float4 pixelColor;
        ///};
        ///
        ///
        /////////////////
        ///// TYPEDEFS //
        /////////////////
        ///struct PixelInputType
        ///{
        ///    float4 position : SV_POSITION;
        ///    float2 tex : TEXCOORD0;
        ///};
        ///
        ///
        ////////////////////////////////////////////////// [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string font {
            get {
                return ResourceManager.GetString("font", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to ////////////////////////////////////////////////////////////////////////////////
        ///// Filename: font.vs
        ///////////////////////////////////////////////////////////////////////////////////
        ///
        ///
        ////////////////
        ///// GLOBALS //
        ////////////////
        ///cbuffer MatrixBuffer
        ///{
        ///	matrix worldMatrix;
        ///	matrix viewMatrix;
        ///	matrix projectionMatrix;
        ///};
        ///
        ///
        /////////////////
        ///// TYPEDEFS //
        /////////////////
        ///struct VertexInputType
        ///{
        ///    float4 position : POSITION;
        ///    float2 tex : TEXCOORD0;
        ///};
        ///
        ///struct PixelInputType
        ///{
        ///    float4 position : SV_POSITIO [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string font1 {
            get {
                return ResourceManager.GetString("font1", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to cbuffer MatrixBuffer :register(b0)
        ///{
        ///	float4x4 worldMatrix;
        ///	float4x4 viewMatrix;
        ///	float4x4 projectionMatrix;
        ///	//float4x4 worldViewProjection;
        ///}
        ///
        ///Texture2D diffuseMap;
        ///SamplerState textureSampler;
        ///
        ///struct VS_INPUT
        ///{
        ///    float4 Pos : POSITION;
        ///	float4 Col : COLOR;
        ///	float2 tex: TEXCOORD;
        ///	//float3 normal : NORMAL;
        ///	//float4 instancePosition : POSITION1;
        ///	//float4 instanceRadRot : POSITION2;
        ///	//float4 instanceRadRotRIGHT : POSITION3;
        ///	//float4 instanceRadRotUP : POSITION4;
        ///};
        ///
        ///struct GS_ [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string HLSL {
            get {
                return ResourceManager.GetString("HLSL", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to Texture2D shaderTexture;
        ///SamplerState SampleType;
        ///
        ///struct PixelInputType
        ///{
        ///    float4 position : SV_POSITION;
        ///    float2 tex : TEXCOORD0;
        ///};
        ///
        ///float4 TexturePixelShader(PixelInputType input) : SV_TARGET
        ///{
        ///	float4 textureColor;
        ///
        ///
        ///    // Sample the pixel color from the texture using the sampler at this texture coordinate location.
        ///    textureColor = shaderTexture.Sample(SampleType, input.tex);
        ///
        ///    return textureColor;
        ///}.
        /// </summary>
        internal static string InstancedTexture {
            get {
                return ResourceManager.GetString("InstancedTexture", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to cbuffer MatrixBuffer
        ///{
        ///	matrix worldMatrix;
        ///	matrix viewMatrix;
        ///	matrix projectionMatrix;
        ///};
        ///
        ///struct VertexInputType
        ///{
        ///    float4 position : POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	float3 instancePosition : TEXCOORD1;
        ///};
        ///
        ///struct PixelInputType
        ///{
        ///    float4 position : SV_POSITION;
        ///    float2 tex : TEXCOORD0;
        ///};
        ///
        ///PixelInputType TextureVertexShader(VertexInputType input)
        ///{
        ///    PixelInputType output;
        ///    
        ///
        ///	// Change the position vector to be 4 units for proper matrix calculations.
        ///    input.position.w = 1.0f;
        ///
        /// [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string InstancedTexture1 {
            get {
                return ResourceManager.GetString("InstancedTexture1", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to ////////////////////////////////////////////////////////////////////////////////
        ///// Filename: texture.ps
        ///////////////////////////////////////////////////////////////////////////////////
        ///
        ///
        ////////////////
        ///// GLOBALS //
        ////////////////
        ///Texture2D shaderTexture;
        ///SamplerState SampleType;
        ///
        ///
        /////////////////
        ///// TYPEDEFS //
        /////////////////
        ///struct PixelInputType
        ///{
        ///    float4 position : SV_POSITION;
        ///    //float2 tex : TEXCOORD0;
        ///	float4 color : COLOR;
        ///};
        ///
        ///
        /////////////////////////////////////////////////////////////////////// [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string red {
            get {
                return ResourceManager.GetString("red", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to ////////////////////////////////////////////////////////////////////////////////
        ///// Filename: texture.vs
        ///////////////////////////////////////////////////////////////////////////////////
        ///
        ///
        ////////////////
        ///// GLOBALS //
        ////////////////
        ///cbuffer MatrixBuffer :register(b0)
        ///{
        ///	//float4x4 worldMatrix;
        ///	//float4x4 viewMatrix;
        ///	//float4x4 projectionMatrix;
        ///	float4x4 worldViewProjection;
        ///}
        ///
        /////cbuffer data :register(b0)
        /////{
        /////	float4x4 worldMatrix;
        /////	float4x4 viewMatrix;
        /////	float4x4 projectionMatrix;        /// [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string red1 {
            get {
                return ResourceManager.GetString("red1", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to Texture2D shaderTexture;
        ///SamplerState SampleType;
        ///
        ///SamplerState textureSampler
        ///{
        ///    Filter = MIN_MAG_MIP_LINEAR;
        ///    AddressU = Wrap;
        ///    AddressV = Wrap;
        ///};
        ///
        ///cbuffer LightBuffer
        ///{
        ///	float4 ambientColor;
        ///	float4 diffuseColor;
        ///	float3 lightDirection;
        ///	float padding0;
        ///	float3 lightPosition;
        ///	float padding1;
        ///};
        ///
        ///struct PixelInputType
        ///{ 
        ///	float4 position : POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	float4 color : COLOR;
        ///	float3 normal : NORMAL;
        ///	float4 instancePosition : POSITION1;
        ///	float [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string texture {
            get {
                return ResourceManager.GetString("texture", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to Texture2D shaderTexture;
        ///SamplerState SampleType;
        ///
        ///SamplerState textureSampler
        ///{
        ///    Filter = MIN_MAG_MIP_LINEAR;
        ///    AddressU = Wrap;
        ///    AddressV = Wrap;
        ///};
        ///
        ///cbuffer LightBuffer
        ///{
        ///	float4 ambientColor;
        ///	float4 diffuseColor;
        ///	float3 lightDirection;
        ///	float padding0;
        ///	float3 lightPosition;
        ///	float padding1;
        ///	int set_texture;
        ///};
        ///
        ///struct PixelInputType
        ///{ 
        ///	float4 position : POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	float4 color : COLOR;
        ///	float3 normal : NORMAL;
        ///	float4 instancePosition : [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string texture___backup {
            get {
                return ResourceManager.GetString("texture___backup", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to Texture2D shaderTexture;
        ///SamplerState SampleType;
        ///
        ///SamplerState textureSampler
        ///{
        ///    Filter = MIN_MAG_MIP_LINEAR;
        ///    AddressU = Wrap;
        ///    AddressV = Wrap;
        ///};
        ///
        ///cbuffer LightBuffer
        ///{
        ///	float4 ambientColor;
        ///	float4 diffuseColor;
        ///	float3 lightDirection;
        ///	float padding0;
        ///	float3 lightPosition;
        ///	float padding1;
        ///};
        ///
        ///struct PixelInputType
        ///{ 
        ///	float4 position : POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	float4 color : COLOR;
        ///	float3 normal : NORMAL;
        ///	float4 instancePosition : POSITION1;
        ///	float [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string texture_voxel {
            get {
                return ResourceManager.GetString("texture_voxel", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to Texture2D shaderTexture;
        ///SamplerState SampleType;
        ///
        ///SamplerState textureSampler
        ///{
        ///    Filter = MIN_MAG_MIP_LINEAR;
        ///    AddressU = Wrap;
        ///    AddressV = Wrap;
        ///};
        ///
        ///cbuffer LightBuffer
        ///{
        ///	float4 ambientColor;
        ///	float4 diffuseColor;
        ///	float3 lightDirection;
        ///	float padding0;
        ///	float3 lightPosition;
        ///	float padding1;
        ///};
        ///
        ///struct PixelInputType
        ///{ 
        ///	float4 position : POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	float4 color : COLOR;
        ///	float3 normal : NORMAL;
        ///	float4 instancePosition : POSITION1;
        ///	float [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string texture_voxel___junk {
            get {
                return ResourceManager.GetString("texture_voxel___junk", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to cbuffer MatrixBuffer : register(b0)
        ///{
        ///	matrix worldMatrix;
        ///	matrix viewMatrix;
        ///	matrix projectionMatrix;
        ///};
        ///
        ///struct VertexInputType
        ///{
        ///    float4 position : POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	float4 color : COLOR;
        ///	float3 normal : NORMAL;
        ///	float4 instancePosition : POSITION1;
        ///	float4 instanceRadRot : POSITION2;
        ///	float4 instanceRadRotRIGHT : POSITION3;
        ///	float4 instanceRadRotUP : POSITION4;
        ///};
        ///
        ///struct PixelInputType
        ///{
        ///    float4 position : SV_POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	fl [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string texture_voxel1 {
            get {
                return ResourceManager.GetString("texture_voxel1", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to cbuffer MatrixBuffer : register(b0)
        ///{
        ///	matrix worldMatrix;
        ///	matrix viewMatrix;
        ///	matrix projectionMatrix;
        ///};
        ///
        ///struct VertexInputType
        ///{
        ///    float4 position : POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	float4 color : COLOR;
        ///	float3 normal : NORMAL;
        ///	float4 instancePosition : POSITION1;
        ///	float4 instanceRadRot : POSITION2;
        ///	float4 instanceRadRotRIGHT : POSITION3;
        ///	float4 instanceRadRotUP : POSITION4;
        ///};
        ///
        ///struct PixelInputType
        ///{
        ///    float4 position : SV_POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	fl [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string texture_voxelPlanet {
            get {
                return ResourceManager.GetString("texture_voxelPlanet", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to Texture2D shaderTexture;
        ///SamplerState SampleType;
        ///
        ///SamplerState textureSampler
        ///{
        ///    Filter = MIN_MAG_MIP_LINEAR;
        ///    AddressU = Wrap;
        ///    AddressV = Wrap;
        ///};
        ///
        ///cbuffer LightBuffer
        ///{
        ///	float4 ambientColor;
        ///	float4 diffuseColor;
        ///	float3 lightDirection;
        ///	float padding0;
        ///	float3 lightPosition;
        ///	float padding1;
        ///};
        ///
        ///struct PixelInputType
        ///{ 
        ///	float4 position : POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	float4 color : COLOR;
        ///	float3 normal : NORMAL;
        ///	float4 instancePosition : POSITION1;
        ///	float [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string texture_voxelPlanet1 {
            get {
                return ResourceManager.GetString("texture_voxelPlanet1", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to cbuffer MatrixBuffer : register(b0)
        ///{
        ///	matrix worldMatrix;
        ///	matrix viewMatrix;
        ///	matrix projectionMatrix;
        ///};
        ///
        ///struct VertexInputType
        ///{
        ///    float4 position : POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	float4 color : COLOR;
        ///	float3 normal : NORMAL;
        ///	float4 instancePosition : POSITION1;
        ///	float4 instanceRadRot : POSITION2;
        ///	float4 instanceRadRotRIGHT : POSITION3;
        ///	float4 instanceRadRotUP : POSITION4;
        ///};
        ///
        ///struct PixelInputType
        ///{
        ///    float4 position : SV_POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	fl [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string texture1 {
            get {
                return ResourceManager.GetString("texture1", resourceCulture);
            }
        }
    }
}
