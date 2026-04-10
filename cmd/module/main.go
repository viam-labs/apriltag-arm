package main

import (
	apriltagarm "github.com/viam-labs/apriltag-arm"

	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
)

func main() {
	module.ModularMain(resource.APIModel{
		API:   generic.API,
		Model: apriltagarm.Model,
	})
}
